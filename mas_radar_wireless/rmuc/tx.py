# -*- coding: utf-8 -*-
"""RMUC2026-NC 发送线程

本模块实现协议节点的发送侧逻辑：
- 从 RuntimeState.tx 读取当前网页发送配置；
- 使用 coder.SerialFrameBuilder 生成 0x0A01 ~ 0x0A06 串口协议帧；
- 将串口帧组织成 15 字节空口 Payload，并封装为 packed bytes 形式的空口帧；
- 通过 sdr_link.SdrLink.send_bytes() 推送给虚拟 SDR 或 GNU Radio ZMQ 数据面；
- 支持随机化发送内容、60 秒信息波偏移/包顺序重置、字节级平滑发送和前端动态配置更新。

设计边界：
- TX 侧只输出 packed bytes，不负责 bit 展开；
- RX unpacked bytes 解码由 rx.py / coder.py 负责，本模块不处理接收链路；
- SDR 频点、功率、调制门控由 sdr_control.py 通过 XMLRPC 同步，本模块只负责数据面；
- RuntimeState.build_sdr_params() 是 SDR 控制参数唯一来源，本模块不直接计算 SDR 控制量。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import os
import random
import string
import threading
import time

from .coder import SerialFrameBuilder, pack_air_frame
from .constants import (
    AIR_FRAME_LEN,
    AIR_PACKETS_PER_CYCLE,
    AIR_PAYLOAD_BYTES_PER_CYCLE,
    AIR_PAYLOAD_LEN,
    WaveType,
)
from .sdr_link import SdrLink, TxChannel
from .state import RuntimeState, clamp_tx_power_dbm


# ==================== 随机化范围 ====================

FIELD_X_RANGE_CM = (0, 2800)
"""随机化机器人 X 坐标范围，单位 cm。"""

FIELD_Y_RANGE_CM = (0, 1500)
"""随机化机器人 Y 坐标范围，单位 cm。"""

ROBOT_HP_RANGE = (0, 1500)
"""随机化机器人血量范围。"""

AMMO_RANGE = (0, 2000)
"""随机化剩余发弹量范围。"""

COIN_RANGE = (0, 2000)
"""随机化金币范围。"""

UINT8_PERCENT_RANGE = (0, 100)
"""随机化百分比类增益范围。"""

SECRET_KEY_ALPHABET = string.ascii_letters + string.digits
"""随机干扰密钥字符集：a-z、A-Z、0-9。"""


# ==================== 平滑发送参数 ====================

SMOOTH_TX_CHUNK_INTERVAL_SEC = 0.10
"""字节级平滑发送间隔，当前每 100 ms 推送一个完整空口周期。"""

SMOOTH_TX_TARGET_BUFFER_SEC = 0.5
"""目标发送缓冲区大小，单位秒。"""

SMOOTH_TX_LOW_BUFFER_SEC = 0.25
"""发送缓冲区低水位线，单位秒。"""

TX_RATE_LOG_INTERVAL_SEC = 1.0
"""TX 实时发送字节数打印周期，单位秒。"""


INFO_STREAM_OFFSET_RESET_SEC = 60.0
"""信息波流式填充偏移重置周期，单位秒。"""


# ==================== 信息波流式填充状态 ====================

@dataclass
class InfoPayloadStreamState:
    """信息波连续 Payload 填充状态。

    信息波不是每个周期独立随机摆放，而是将 0x0A01~0x0A05 串口帧字节视为
    连续字节流。每个发送周期取 135 字节覆盖 9 个空口包 Payload；未放入当前
    周期的尾部字节会保留到下一个周期继续填入。

    每 60 秒重置一次流状态：
    - 清空 carry_bytes；
    - 生成 0~14 字节随机偏置填充，该偏置只影响重置后的第一个周期；
    - 完全随机打乱 0x0A01~0x0A05 的拼接顺序，并保持到下一次重置。
    """

    carry_bytes: bytes
    expires_at: float
    frame_order: tuple[str, ...]


# ==================== TX 工作线程 ====================

class TxWorker:
    """发送线程：生成空口帧 packed bytes，并以小块形式推送到 SDR 数据链路。

    发送策略：
    - 信息波承载 0x0A01 ~ 0x0A05，每 60 秒完全随机重置拼接顺序；
    - 干扰波承载 0x0A06，每个周期在前 15 字节内随机选择 SOF 偏移位置；
    - 一个发送周期包含 9 个空口包，每个空口包承载 15 字节 Payload；
    - 内部维护 packed bytes 缓冲区，按 10 ms 小块平滑输出，避免一次性突发大块数据。
    """

    def __init__(self, state: RuntimeState, sdr_link: SdrLink, rate_hz: float = 10.0):
        """初始化发送线程。

        参数:
            state: 全局运行状态，TX 配置和计数器均从这里读写。
            sdr_link: SDR 数据面链路，负责发送 packed bytes。
            rate_hz: 软件发送周期频率，默认 10 Hz。
        """
        if rate_hz <= 0:
            raise ValueError("rate_hz must be positive")

        self.state = state
        self.sdr_link = sdr_link
        self.rate_hz = float(rate_hz)
        self.serial_frame_builder = SerialFrameBuilder()

        self._info_stream_state: InfoPayloadStreamState | None = None

        self._byte_buffer: deque[list[tuple[TxChannel, WaveType, bytes]]] = deque()
        self._byte_buffer_size = 0

        # 速率控制：
        # 一个软件发送周期包含 9 个空口帧，每个空口帧 27 byte。
        # 默认 rate_hz=10 时，每 100 ms 发送 243 byte。
        self._cycle_chunk_bytes = AIR_FRAME_LEN * AIR_PACKETS_PER_CYCLE
        self._target_rate_bytes_per_sec = self._cycle_chunk_bytes * self.rate_hz
        self._chunk_interval_sec = 1.0 / self.rate_hz
        self._target_buffer_bytes = int(
            self._target_rate_bytes_per_sec * SMOOTH_TX_TARGET_BUFFER_SEC
        )
        self._low_buffer_bytes = int(
            self._target_rate_bytes_per_sec * SMOOTH_TX_LOW_BUFFER_SEC
        )

        # TX 实时发送统计，用于观察 ZMQ 推流是否稳定。
        now = time.perf_counter()
        self._rate_log_start_time = now
        self._rate_log_bytes = 0
        self._rate_log_chunks = 0

        self._buffer_lock = threading.RLock()
        self._thread: threading.Thread | None = None
        self._running = False

    # ==================== 生命周期 ====================

    def start(self) -> None:
        """启动 TX 后台线程。"""
        if self._running:
            return

        self._running = True
        self._thread = threading.Thread(target=self._run, name="rmuc-tx", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """停止 TX 后台线程。"""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2)
            self._thread = None

    # ==================== 主循环 ====================

    def _run(self) -> None:
        """发送主循环：维护缓冲区，并按固定间隔发送周期对齐字节块。"""
        next_send_time = time.perf_counter()

        while self._running:
            self._ensure_byte_buffer()

            now = time.perf_counter()
            if now >= next_send_time:
                cycle_chunks = self._next_cycle_chunks()
                for tx_channel, wave_type, chunk_bytes in cycle_chunks:
                    self._transmit_bytes(tx_channel, wave_type, chunk_bytes)

                next_send_time += self._chunk_interval_sec
                if next_send_time < now - self._chunk_interval_sec:
                    next_send_time = now + self._chunk_interval_sec

            sleep_sec = min(0.002, max(0.0, next_send_time - time.perf_counter()))
            if sleep_sec > 0:
                time.sleep(sleep_sec)

    # ==================== 字节缓冲区 ====================

    def _ensure_byte_buffer(self) -> None:
        """保证发送缓冲区维持在目标水位附近。"""
        with self._buffer_lock:
            if self._byte_buffer_size >= self._low_buffer_bytes:
                return

            while self._byte_buffer_size < self._target_buffer_bytes:
                cycle_chunks = self._generate_next_air_frame_chunks()
                self._byte_buffer.append(cycle_chunks)
                self._byte_buffer_size += sum(
                    len(air_frame_chunk)
                    for _tx_channel, _wave_type, air_frame_chunk in cycle_chunks
                )

                self.state.counters.air_frames_tx += sum(
                    len(air_frame_chunk) // AIR_FRAME_LEN
                    for _tx_channel, _wave_type, air_frame_chunk in cycle_chunks
                )

    def _next_cycle_chunks(self) -> list[tuple[TxChannel, WaveType, bytes]]:
        """从发送缓冲区头部取出一个周期内所有物理源的小块。

        当前默认参数下：
            AIR_FRAME_LEN = 27
            AIR_PACKETS_PER_CYCLE = 9
            单个源 chunk_bytes = 243

        双源模式下同一个 100 ms 周期会同时推送主源和副源，避免两个端口交替欠供。
        """
        with self._buffer_lock:
            if not self._byte_buffer:
                return []

            cycle_chunks = self._byte_buffer.popleft()
            self._byte_buffer_size -= sum(
                len(chunk) for _tx_channel, _wave_type, chunk in cycle_chunks
            )
            return cycle_chunks

    def _transmit_bytes(self, tx_channel: TxChannel, wave_type: WaveType, chunk: bytes) -> None:
        """发送一个 packed bytes 周期块，并实时显示发送字节数。"""
        if not chunk:
            return

        chunk_len = len(chunk)
        self.state.counters.tx_bytes_sent += chunk_len
        self.state.counters.tx_bits_sent += chunk_len * 8
        self.sdr_link.send_bytes(wave_type, chunk, tx_channel=tx_channel)

        self._rate_log_bytes += chunk_len
        self._rate_log_chunks += 1

        now = time.perf_counter()
        elapsed_sec = now - self._rate_log_start_time
        if elapsed_sec >= TX_RATE_LOG_INTERVAL_SEC:
            byte_rate = self._rate_log_bytes / elapsed_sec
            chunk_rate = self._rate_log_chunks / elapsed_sec
            print(
                "TX sent: "
                f"{self.state.counters.tx_bytes_sent} B total, "
                f"{byte_rate:.1f} B/s, "
                f"{chunk_rate:.1f} chunks/s, "
                f"last_chunk={chunk_len} B, "
                f"buffer={self._byte_buffer_size} B",
                flush=True,
            )

            self._rate_log_start_time = now
            self._rate_log_bytes = 0
            self._rate_log_chunks = 0

    def _clear_generated_data(self, *, clear_buffer: bool) -> None:
        """清空已经生成但尚未发送的数据状态和可选字节缓冲区。"""
        self._info_stream_state = None
        if clear_buffer:
            self._byte_buffer.clear()
            self._byte_buffer_size = 0
            self._rate_log_start_time = time.perf_counter()
            self._rate_log_bytes = 0
            self._rate_log_chunks = 0

    # ==================== 空口帧生成 ====================

    def _generate_next_air_frame_chunks(self) -> list[tuple[TxChannel, WaveType, bytes]]:
        """生成下一个发送周期的空口帧块。

        单源模式只生成当前默认源；双源模式下默认源固定为干扰波，第二源固定为信息波。
        每个返回项都带有 tx_channel，后续真实链路按物理源端口分流。
        """
        primary_wave_type = self.state.tx.wave_type
        dual_source = (
            self.state.tx.second_source_enabled
            and primary_wave_type == WaveType.INTERFERE
        )

        if self.state.tx.randomize:
            # 随机发送模式用于 GUI/协议调试；双源时需要同时刷新干扰波和信息波业务内容。
            self._randomize_tx_state(randomize_info=dual_source)

        tx_chunks = [(TxChannel.PRIMARY, self._build_air_frame_chunk(primary_wave_type))]
        if dual_source:
            tx_chunks.append((TxChannel.SECONDARY, self._build_air_frame_chunk(WaveType.INFO)))

        self.state.counters.serial_frames_tx += sum(
            serial_frame_count
            for _tx_channel, (_wave_type, _chunk, serial_frame_count) in tx_chunks
        )
        return [
            (tx_channel, wave_type, chunk)
            for tx_channel, (wave_type, chunk, _serial_frame_count) in tx_chunks
        ]

    def _build_air_frame_chunk(self, wave_type: WaveType) -> tuple[WaveType, bytes, int]:
        """构造单个波形支路的一个完整空口周期块。"""
        payload_cycle, serial_frame_count = self._build_next_payload_cycle(wave_type)
        air_frames = [
            pack_air_frame(wave_type, payload_cycle[offset : offset + AIR_PAYLOAD_LEN])
            for offset in range(0, len(payload_cycle), AIR_PAYLOAD_LEN)
        ]
        return wave_type, b"".join(air_frames), serial_frame_count

    def _build_next_payload_cycle(self, wave_type: WaveType) -> tuple[bytes, int]:
        """构造下一个 135 字节 Payload 周期。"""
        if wave_type == WaveType.INFO:
            return self._build_info_payload_cycle(time.perf_counter())

        return self._build_interference_payload_cycle()

    def _build_info_payload_cycle(self, now: float) -> tuple[bytes, int]:
        """构造信息波 135 字节 Payload 周期。

        算法：
        1. 每 60 秒重置信息流状态；
        2. 重置时清空上一轮 carry，并加入 0~14 字节随机偏移填充；
        3. 重置时完全随机打乱 0x0A01~0x0A05 的拼接顺序；
        4. 在 60 秒窗口内按该随机顺序拼接当前周期串口帧；
        5. 将 carry + 当前信息流组成至多约 150 字节的新数组；
        6. 前 135 字节覆盖本周期 9 个空口包 Payload；
        7. 剩余字节保留到下一个周期继续填入。
        """
        if self._info_stream_state is None or now >= self._info_stream_state.expires_at:
            initial_offset = random.randrange(AIR_PAYLOAD_LEN)
            self._info_stream_state = InfoPayloadStreamState(
                carry_bytes=os.urandom(initial_offset),
                expires_at=now + INFO_STREAM_OFFSET_RESET_SEC,
                frame_order=self._random_info_frame_order(),
            )

        serial_frames = self._build_info_serial_frames(self._info_stream_state.frame_order)
        ordered_info_bytes = b"".join(serial_frames)

        combined = self._info_stream_state.carry_bytes + ordered_info_bytes

        payload_cycle = bytearray(os.urandom(AIR_PAYLOAD_BYTES_PER_CYCLE))
        copy_len = min(AIR_PAYLOAD_BYTES_PER_CYCLE, len(combined))
        if copy_len:
            payload_cycle[:copy_len] = combined[:copy_len]

        self._info_stream_state.carry_bytes = combined[AIR_PAYLOAD_BYTES_PER_CYCLE:]

        return bytes(payload_cycle), len(serial_frames)

    def _build_interference_payload_cycle(self) -> tuple[bytes, int]:
        """构造干扰波 135 字节 Payload 周期。

        每个周期先生成随机 Payload，再在前 15 字节中随机选择一个 SOF 偏移位置，
        将 0x0A06 串口帧覆盖写入该位置。
        """
        payload_cycle = bytearray(os.urandom(AIR_PAYLOAD_BYTES_PER_CYCLE))
        serial_frames = self._build_interference_serial_frames()

        if serial_frames:
            serial_frame = serial_frames[0]
            sof_offset = random.randrange(AIR_PAYLOAD_LEN)
            copy_len = min(len(serial_frame), AIR_PAYLOAD_BYTES_PER_CYCLE - sof_offset)
            payload_cycle[sof_offset : sof_offset + copy_len] = serial_frame[:copy_len]

        return bytes(payload_cycle), len(serial_frames)

    # ==================== 串口帧构造 ====================

    def _build_info_serial_frames(self, frame_order: tuple[str, ...]) -> list[bytes]:
        """根据 60 秒窗口内的随机顺序构造信息波串口帧列表。"""
        tx = self.state.tx
        frames: list[bytes] = []

        for cmd_id in frame_order:
            if not tx.enabled_cmds.get(cmd_id, True):
                continue

            if cmd_id == "0x0A01":
                frames.append(self.serial_frame_builder.robot_position_0x0A01(tx.positions))
            elif cmd_id == "0x0A02":
                frames.append(self.serial_frame_builder.robot_hp_0x0A02(tx.hps))
            elif cmd_id == "0x0A03":
                frames.append(self.serial_frame_builder.robot_ammo_0x0A03(tx.ammo))
            elif cmd_id == "0x0A04":
                frames.append(
                    self.serial_frame_builder.field_status_0x0A04(
                        tx.remain_coins,
                        tx.total_coins,
                        tx.status_flags,
                    )
                )
            elif cmd_id == "0x0A05":
                frames.append(
                    self.serial_frame_builder.robot_buff_0x0A05(
                        tx.buff_hero,
                        tx.buff_eng,
                        tx.buff_inf3,
                        tx.buff_inf4,
                        tx.buff_sentry,
                        tx.robot_statuses,
                    )
                )

        return frames

    @staticmethod
    def _random_info_frame_order() -> tuple[str, ...]:
        """生成 0x0A01~0x0A05 的完全随机拼接顺序。"""
        frame_order = ["0x0A01", "0x0A02", "0x0A03", "0x0A04", "0x0A05"]
        random.shuffle(frame_order)
        return tuple(frame_order)

    def _build_interference_serial_frames(self) -> list[bytes]:
        """根据当前 TX 状态构造干扰波串口帧列表。"""
        tx = self.state.tx
        if not tx.enabled_cmds.get("0x0A06", True):
            return []

        return [self.serial_frame_builder.interference_key_0x0A06(tx.secret_key)]

    # ==================== 随机化发送内容 ====================

    def _randomize_tx_state(self, *, randomize_info: bool = False) -> None:
        """随机修改当前 TX payload 数据，用于 GUI 调试和链路压力测试。"""
        tx = self.state.tx

        if tx.wave_type == WaveType.INTERFERE:
            tx.secret_key = "".join(random.choice(SECRET_KEY_ALPHABET) for _ in range(6))
            if not randomize_info:
                return

        tx.positions = [
            (random.randint(*FIELD_X_RANGE_CM), random.randint(*FIELD_Y_RANGE_CM))
            for _ in range(6)
        ]
        tx.hps = [random.randint(*ROBOT_HP_RANGE) for _ in range(5)]
        tx.ammo = [random.randint(*AMMO_RANGE) for _ in range(5)]
        tx.remain_coins = random.randint(*COIN_RANGE)
        tx.total_coins = random.randint(tx.remain_coins, COIN_RANGE[1])
        tx.status_flags = random.randint(0, 0xFFFF)
        tx.buff_hero = self._random_buff()
        tx.buff_eng = self._random_buff()
        tx.buff_inf3 = self._random_buff()
        tx.buff_inf4 = self._random_buff()
        tx.buff_sentry = self._random_buff() + [random.randint(1, 6)]
        tx.robot_statuses = [random.randint(0, 3) for _ in range(5)]

    @staticmethod
    def _random_buff() -> list[int]:
        """生成单个机器人随机增益状态。"""
        return [
            random.randint(0, 1),
            random.randint(*UINT8_PERCENT_RANGE),
            random.randint(*UINT8_PERCENT_RANGE),
            random.randint(*UINT8_PERCENT_RANGE),
            random.randint(*UINT8_PERCENT_RANGE),
        ]

    # ==================== GUI 更新接口 ====================

    def update_tx(
        self,
        patch: dict,
        *,
        reset_source_on_category_change: bool = True,
        clear_on_payload_change: bool = True,
    ) -> bool:
        """更新发送配置。

        参数:
            patch: GUI 或 node 传入的配置变更字典。
            reset_source_on_category_change: 发送源类别变化时是否清空状态和缓冲区。
            clear_on_payload_change: payload 数据变化时是否清空状态和缓冲区。

        返回:
            True 表示发送源类别发生变化，例如信息波切到干扰波或干扰等级改变。
        """
        with self._buffer_lock:
            old_source_category = self._source_category()

            self._apply_tx_control_patch(patch)
            payload_changed = self._apply_tx_payload_patch(patch)

            source_changed = old_source_category != self._source_category()

            if reset_source_on_category_change and source_changed:
                self._clear_generated_data(clear_buffer=True)
            elif clear_on_payload_change and payload_changed:
                self._clear_generated_data(clear_buffer=True)

        self.state.log("TX 配置已更新")
        return source_changed

    def _apply_tx_control_patch(self, patch: dict) -> None:
        """应用 TX 控制字段更新。"""
        tx = self.state.tx

        if "team" in patch:
            tx.team = type(tx.team)(patch["team"])
        if "wave_type" in patch:
            tx.wave_type = WaveType(patch["wave_type"])
        if "interference_level" in patch:
            tx.interference_level = max(1, min(3, int(patch["interference_level"])))
        if "tx_power_dbm" in patch:
            tx.tx_power_dbm = clamp_tx_power_dbm(patch["tx_power_dbm"])
        if "second_source_enabled" in patch:
            # 第二源开启后，主源固定干扰波，副源固定信息波，并由 TX 数据链路分端口发送。
            tx.second_source_enabled = bool(patch["second_source_enabled"])
        if "second_tx_power_dbm" in patch:
            tx.second_tx_power_dbm = clamp_tx_power_dbm(patch["second_tx_power_dbm"])

        if tx.second_source_enabled and tx.wave_type == WaveType.INFO:
            # 双源模式下主源固定承载干扰波，副源固定承载信息波；这里兜底修正外部 API 的非法组合。
            tx.wave_type = WaveType.INTERFERE
            tx.interference_level = max(1, min(3, int(tx.interference_level)))

    def _apply_tx_payload_patch(self, patch: dict) -> bool:
        """应用 TX payload 字段更新。

        返回:
            True 表示本次 patch 修改了业务 payload 或随机化状态。
        """
        tx = self.state.tx
        payload_changed = False

        if "randomize" in patch:
            tx.randomize = bool(patch["randomize"])
            payload_changed = True

        if "enabled_cmds" in patch:
            for cmd, enabled in dict(patch["enabled_cmds"]).items():
                if cmd in tx.enabled_cmds:
                    tx.enabled_cmds[cmd] = bool(enabled)
                    payload_changed = True

        if "secret_key" in patch:
            tx.secret_key = self._normalize_secret_key(patch["secret_key"])
            payload_changed = True

        if "positions" in patch:
            positions = self._parse_positions(patch["positions"])
            if positions is not None:
                tx.positions = positions
                payload_changed = True

        if "hps" in patch:
            hps = self._parse_int_list(patch["hps"], expected_len=5)
            if hps is not None:
                tx.hps = hps
                payload_changed = True

        if "ammo" in patch:
            ammo = self._parse_int_list(patch["ammo"], expected_len=5)
            if ammo is not None:
                tx.ammo = ammo
                payload_changed = True

        if "buffs" in patch:
            if self._apply_buffs_patch(dict(patch["buffs"])):
                payload_changed = True

        if "remain_coins" in patch:
            tx.remain_coins = int(patch["remain_coins"])
            payload_changed = True

        if "total_coins" in patch:
            tx.total_coins = int(patch["total_coins"])
            payload_changed = True

        if "status_flags" in patch:
            tx.status_flags = int(str(patch["status_flags"]), 0)
            payload_changed = True

        if "robot_statuses" in patch:
            parsed_statuses = self._parse_int_list(patch["robot_statuses"], expected_len=5)
            if parsed_statuses is not None:
                tx.robot_statuses = parsed_statuses
                payload_changed = True

        return payload_changed

    def _apply_buffs_patch(self, buffs: dict) -> bool:
        """应用 0x0A05 增益状态字段更新。"""
        changed = False
        for attr_name, expected_len in (
            ("buff_hero", 5),
            ("buff_eng", 5),
            ("buff_inf3", 5),
            ("buff_inf4", 5),
            ("buff_sentry", 6),
        ):
            values = buffs.get(attr_name)
            parsed = self._parse_int_list(values, expected_len=expected_len)
            if parsed is not None:
                setattr(self.state.tx, attr_name, parsed)
                changed = True

        return changed

    @staticmethod
    def _normalize_secret_key(raw_key) -> str:
        """规范化 0x0A06 密钥：保留 ASCII 字母大小写和数字，不足 6 位补 0。"""
        allowed_chars = set(SECRET_KEY_ALPHABET)
        normalized = "".join(char for char in str(raw_key) if char in allowed_chars)
        return normalized[:6].ljust(6, "0")

    @staticmethod
    def _parse_positions(raw_positions) -> list[tuple[int, int]] | None:
        """解析前端 positions 字段，兼容 dict 和二元序列两种格式。"""
        if not isinstance(raw_positions, list) or len(raw_positions) != 6:
            return None

        parsed: list[tuple[int, int]] = []
        for item in raw_positions:
            if isinstance(item, dict):
                parsed.append((int(item.get("x", 0)), int(item.get("y", 0))))
            elif isinstance(item, (list, tuple)) and len(item) >= 2:
                parsed.append((int(item[0]), int(item[1])))
            else:
                return None

        return parsed

    @staticmethod
    def _parse_int_list(raw_values, *, expected_len: int) -> list[int] | None:
        """解析固定长度整数列表。"""
        if not isinstance(raw_values, list) or len(raw_values) != expected_len:
            return None

        return [int(value) for value in raw_values]

    def _source_category(self) -> tuple[str, int, int]:
        """返回影响空口数据源类别的关键信息。"""
        tx = self.state.tx
        if tx.wave_type == WaveType.INFO:
            return (tx.wave_type.value, 0, int(bool(tx.second_source_enabled)))
        return (
            tx.wave_type.value,
            max(1, min(3, int(tx.interference_level))),
            int(bool(tx.second_source_enabled)),
        )


__all__ = [
    "InfoPayloadStreamState",
    "TxWorker",
]
