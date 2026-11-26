# -*- coding: utf-8 -*-
"""RMUC2026-NC 接收线程与协议解码模块

本模块只负责 RX 数据面处理，不再包含通道选择器或自动能量选择逻辑。

设计边界：
- RX 解调模式由网页强制选择，并由 node.py 写入 RuntimeState.rx_selection；
- SDR 频点、匹配滤波门控等控制参数由 RuntimeState.build_sdr_params() 计算，
  再由 sdr_control.py 通过 XMLRPC 同步到 GNU Radio；
- sdr_link.py 只返回 GNU Radio 风格的 unpacked bytes；
- rx.py 只负责 unpacked bytes -> 空口帧 -> 串口帧 -> 业务数据 -> RuntimeState.rx。

接收链路：
    SdrLink.recv_unpacked_bytes()
    -> AirFrameDecoder.feed_unpacked_bytes()
    -> SerialFrameDecoder.feed_events()
    -> BusinessDecoder.decode()
    -> RuntimeState.rx / packets_by_cmd / cmd_stats / rx_diag_stats
"""

from __future__ import annotations

import threading
import time
from typing import Callable

from .coder import (
    AirFrameDecodeEvent,
    AirFrameDecoder,
    AirFrameRecord,
    BusinessDecoder,
    SerialFrame,
    SerialFrameDecoder,
    classify_serial_frame_bytes,
)
from .constants import WaveType
from .sdr_link import SdrLink
from .state import RuntimeState, parse_interference_key, rx_profiles_from_selection


# ==================== 类型定义 ====================

DecodedPacketCallback = Callable[[str, dict], None]
"""串口业务包 CRC 校验通过并写入 RuntimeState 后触发的回调。"""


# ==================== RX 工作线程 ====================

class RxWorker:
    """接收线程：从 SDR 数据链路读取 unpacked bytes，恢复空口帧并解析业务数据。

    本类不负责选择接收频点，也不负责决定当前应解红方还是蓝方。
    这些控制面状态由 RuntimeState.rx_selection 保存，并由 node.py / sdr_control.py 应用。

    本类只根据 sdr_link.py 返回的 wave_type 将数据送入对应解码器：
    - WaveType.INTERFERE：干扰波支路；
    - WaveType.INFO：信息波支路，仅在当前强制选择为 3 级时接收。
    """

    _IDLE_SLEEP_SEC = 0.02
    """SDR 链路暂无数据时的接收线程休眠时间，避免 CPU 空转。"""

    _ERROR_LOG_MIN_INTERVAL_SEC = 0.5
    """同类高频错误日志的最小输出间隔，避免 GUI 日志区刷屏。"""

    def __init__(self, state: RuntimeState, sdr_link: SdrLink):
        self.state = state
        self.sdr_link = sdr_link

        # 信息波与干扰波必须使用独立解码器，避免两个支路的滑窗缓存互相污染。
        self.air_decoders: dict[WaveType, AirFrameDecoder] = {
            wave_type: AirFrameDecoder(wave_type)
            for wave_type in WaveType
        }
        self.serial_decoders: dict[WaveType, SerialFrameDecoder] = {
            wave_type: SerialFrameDecoder()
            for wave_type in WaveType
        }

        # 记录连续 payload 字节与网页空口包字典的引用关系，用于串口帧跨空口包时回填颜色标签。
        self._rx_payload_refs: dict[WaveType, list[tuple[dict, int]]] = {
            wave_type: []
            for wave_type in WaveType
        }

        self._last_error_log_time: dict[str, float] = {}
        self._decoded_callbacks: list[DecodedPacketCallback] = []
        self._thread: threading.Thread | None = None
        self._running = False

    # ==================== 生命周期 ====================

    def start(self) -> None:
        """启动 RX 后台线程。"""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, name="rmuc-rx", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """停止 RX 后台线程。"""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2)
            self._thread = None

    def add_decoded_callback(self, callback: DecodedPacketCallback) -> None:
        """注册串口业务包解码成功回调。

        回调触发时机为：串口帧 CRC 校验通过、业务字段写入 RuntimeState、
        GUI 空口包缓存完成后。ROS2 发布器可借此做到每次解包成功立即发布。
        """
        self._decoded_callbacks.append(callback)

    # ==================== 主循环 ====================

    def _run(self) -> None:
        """接收主循环：轮询 SDR 链路获取 unpacked bytes。"""
        while self._running:
            chunk = self.sdr_link.recv_unpacked_bytes()
            if chunk is None:
                time.sleep(self._IDLE_SLEEP_SEC)
                continue

            self.state.counters.rx_unpacked_bytes += len(chunk.unpacked_bytes)
            self.feed_unpacked_bytes(chunk.wave_type, chunk.unpacked_bytes)

    # ==================== 空口帧入口 ====================

    def feed_unpacked_bytes(
        self,
        wave_type: WaveType | None,
        unpacked_bytes: bytes,
    ) -> None:
        """喂入一段 SDR 解调得到的 unpacked bytes。

        参数:
            wave_type: 数据来源支路。真实 ZMQ 链路由端口确定；虚拟链路由 TX 写入时保留。
            unpacked_bytes: GNU Radio 风格 unpacked bytes，每个 byte 表示一个判决 bit。
        """
        if wave_type is None:
            self.state.log("RX 收到未标记 wave_type 的 unpacked bytes，已丢弃。")
            return

        if not self._should_accept_wave_type(wave_type):
            return

        decoder = self.air_decoders[wave_type]
        air_records = decoder.feed_unpacked_bytes(unpacked_bytes)
        self._log_air_events(decoder.pop_events())

        if air_records:
            self.state.counters.air_payloads_rx += len(air_records)
            for _record in air_records:
                self._record_rx_diag_event("air_header_ok")

        for air_record in air_records:
            self._feed_air_record(wave_type, air_record)

    def _should_accept_wave_type(self, wave_type: WaveType) -> bool:
        """判断当前软件层是否接收该波形支路。

        实际硬件支路门控由 GNU Radio 和 SDR 参数决定；这里保留一层轻量软件门控，
        主要用于 debug/虚拟链路场景，避免非 3 级模式下的信息波数据污染 GUI 状态。
        """
        if wave_type == WaveType.INTERFERE:
            return True

        if wave_type == WaveType.INFO:
            _team, level = parse_interference_key(self.state.rx_selection.interference_current)
            return level == 3

        return False

    # ==================== 空口帧 -> 串口帧 ====================

    def _feed_air_record(self, wave_type: WaveType, air_record: AirFrameRecord) -> None:
        """处理一个成功恢复的空口帧。"""
        packet_dict = air_record.to_dict()
        packet_dict["decoded"] = []

        refs = self._rx_payload_refs[wave_type]
        for payload_index, _payload_byte in enumerate(air_record.payload):
            refs.append((packet_dict, payload_index))

        serial_decoder = self.serial_decoders[wave_type]
        events = serial_decoder.feed_events(air_record.payload)

        for event in events:
            if event.kind == "skip":
                self._trim_rx_payload_refs(wave_type, event.consumed)
                continue

            if event.kind == "error":
                self._trim_rx_payload_refs(wave_type, event.consumed)
                self._log_serial_decode_failure(event.cmd_id, event.reason)
                continue

            if event.frame is None:
                continue

            self._handle_serial_frame(wave_type, event.frame, event.consumed)

    def _handle_serial_frame(
        self,
        wave_type: WaveType,
        serial_frame: SerialFrame,
        consumed_bytes: int,
    ) -> None:
        """处理一个通过 CRC 校验的串口协议帧。"""
        self._record_rx_diag_event("crc16_ok")
        self.state.counters.serial_frames_rx += 1

        frame_start_index = consumed_bytes - len(serial_frame.raw)
        packets = self._label_serial_frame_bytes(wave_type, serial_frame, frame_start_index)
        self._trim_rx_payload_refs(wave_type, consumed_bytes)

        decoded = BusinessDecoder.decode(serial_frame)
        command = decoded.get("cmd", "unknown")

        for packet in packets:
            packet["decoded"].append(command)

        self._apply_decoded_business_data(decoded)
        self._remember_packets_for_gui(command, wave_type, packets, decoded)
        self._emit_decoded_packet(command, decoded)

    def _emit_decoded_packet(self, command: str, decoded: dict) -> None:
        """通知外部模块某个业务包已经成功解码。"""
        for callback in list(self._decoded_callbacks):
            try:
                callback(command, decoded)
            except Exception as exc:
                self.state.log(f"RX 解包回调失败 {command}: {exc}")

    def _label_serial_frame_bytes(
        self,
        wave_type: WaveType,
        serial_frame: SerialFrame,
        frame_start_index: int,
    ) -> list[dict]:
        """根据串口帧字节分类结果，回填空口包 payload 字节颜色标签。"""
        labels = classify_serial_frame_bytes(serial_frame.raw)
        refs = self._rx_payload_refs[wave_type]

        packets: list[dict] = []
        seen_packet_ids: set[int] = set()

        for offset, label in enumerate(labels):
            ref_index = frame_start_index + offset
            if ref_index < 0 or ref_index >= len(refs):
                continue

            packet, payload_index = refs[ref_index]
            packet["payload"][payload_index]["label"] = label

            packet_id = id(packet)
            if packet_id not in seen_packet_ids:
                seen_packet_ids.add(packet_id)
                packets.append(packet)

        return packets

    def _trim_rx_payload_refs(self, wave_type: WaveType, consumed_bytes: int) -> None:
        """删除已经被串口滑窗解码器消费的 payload 字节引用。"""
        if consumed_bytes > 0:
            del self._rx_payload_refs[wave_type][:consumed_bytes]

    # ==================== GUI 空口包记录 ====================

    def _remember_packets_for_gui(
        self,
        command: str,
        wave_type: WaveType,
        packets: list[dict],
        decoded: dict,
    ) -> None:
        """保存最新一次串口命令包及其长度状态，供网页空口包区域显示。"""
        self.state.packets_by_cmd[command] = {
            "cmd": command,
            "wave_type": wave_type.value,
            "frequency_mhz": self._current_branch_frequency_mhz(wave_type),
            "payload_len": int(decoded.get("payload_len", 0)),
            "length_valid": bool(decoded.get("length_valid", False)),
            "air_packets": packets,
        }

    def _current_branch_frequency_mhz(self, wave_type: WaveType) -> float:
        """获取当前支路对应的显示频点，单位 MHz。"""
        interference_profile, info_profile = rx_profiles_from_selection(self.state.rx_selection)
        profile = info_profile if wave_type == WaveType.INFO else interference_profile
        return round(int(profile["freq_hz"]) / 1e6, 3)

    # ==================== 业务数据更新 ====================

    def _apply_decoded_business_data(self, decoded: dict) -> None:
        """将业务层解码结果写入 RuntimeState.rx，并标记 GUI 高亮字段。"""
        command = decoded.get("cmd", "unknown")
        self._update_command_stats(command)

        updated_keys: list[str] = []

        if command == "0x0A01":
            self.state.rx.setdefault("robots", {})
            for robot_name, position in decoded["positions"].items():
                self.state.rx["robots"].setdefault(robot_name, {}).update(position)
                updated_keys.extend([f"robots.{robot_name}.x", f"robots.{robot_name}.y"])

        elif command == "0x0A02":
            self.state.rx.setdefault("robots", {})
            for robot_name, hp in decoded["hp"].items():
                self.state.rx["robots"].setdefault(robot_name, {})["hp"] = hp
                updated_keys.append(f"robots.{robot_name}.hp")

        elif command == "0x0A03":
            self.state.rx.setdefault("robots", {})
            for robot_name, ammo in decoded["ammo"].items():
                self.state.rx["robots"].setdefault(robot_name, {})["ammo"] = ammo
                updated_keys.append(f"robots.{robot_name}.ammo")

        elif command == "0x0A04":
            self.state.rx["field"] = decoded["field"]
            updated_keys.extend([
                "field.remain_coins",
                "field.total_coins",
                "field.status_flags",
            ])

        elif command == "0x0A05":
            self.state.rx.setdefault("robots", {})
            for robot_name, buff in decoded["buffs"].items():
                self.state.rx["robots"].setdefault(robot_name, {}).update(buff)
                updated_keys.extend([f"robots.{robot_name}.{key}" for key in buff])
            self.state.rx["sentry_posture"] = decoded["sentry_posture"]
            updated_keys.append("sentry_posture")
            for robot_name, status in decoded["robot_statuses"].items():
                self.state.rx["robots"].setdefault(robot_name, {})["status"] = status
                updated_keys.append(f"robots.{robot_name}.status")

        elif command == "0x0A06":
            self.state.rx["secret_key"] = decoded["key"]
            updated_keys.append("secret_key")

        self.state.mark_updated(updated_keys)
        payload_len = int(decoded.get("payload_len", 0))
        if decoded.get("length_valid", False):
            self.state.log(f"RX 解包成功 {command}，包长 {payload_len} B")
        else:
            self.state.log(f"RX 包长异常 {command}，包长 {payload_len} B，缺失字段已补 0")

    def _update_command_stats(self, command: str) -> None:
        """更新包 ID 接收统计，用于网页显示接收频率。"""
        now = time.perf_counter()
        stats = self.state.cmd_stats.setdefault(
            command,
            {
                "count": 0,
                "last_time": None,
                "rate_hz": 0.0,
                "events": [],
            },
        )

        last_time = stats.get("last_time")
        stats["count"] += 1
        stats.setdefault("events", []).append(now)

        if last_time is not None:
            elapsed_sec = now - last_time
            if elapsed_sec > 0:
                stats["rate_hz"] = 1.0 / elapsed_sec

        stats["last_time"] = now

    # ==================== 诊断与日志 ====================

    def _log_serial_decode_failure(self, command_id: int | None, reason: str | None) -> None:
        """记录串口帧解析失败事件。"""
        if reason not in ("CRC8", "CRC16"):
            return

        command_text = "未知" if command_id is None else f"0x{command_id:04X}"
        self.state.counters.serial_frames_failed += 1

        if reason == "CRC16":
            stats = self._record_rx_diag_event("crc16_fail")
            self.state.log(
                f"RX {command_text} 包解析失败：CRC16 错误，失败频率 "
                f"{stats.get('rate_hz', 0.0):.1f} Hz"
            )
        else:
            self.state.log(f"RX {command_text} 包解析失败：CRC8 错误")

    def _log_air_events(self, events: list[AirFrameDecodeEvent]) -> None:
        """记录空口帧诊断事件。"""
        for event in events:
            if event.kind != "header_error":
                continue

            stats = self._record_rx_diag_event("air_header_fail")
            log_key = f"air_header:{event.wave_type.value}"
            if not self._should_log_error(log_key, self._ERROR_LOG_MIN_INTERVAL_SEC):
                continue

            polarity_text = "，整帧反相已校正" if event.inverted else ""
            access_errors = getattr(event, "access_errors", 0)
            self.state.log(
                f"RX {self._wave_label(event.wave_type)} 空口帧头错误："
                f"Access Code 已匹配，误码 {access_errors} bit，"
                f"Header={event.header_hex}{polarity_text}，"
                f"失败频率 {stats.get('rate_hz', 0.0):.1f} Hz"
            )

    def _record_rx_diag_event(self, key: str) -> dict:
        """记录 RX 诊断事件并刷新诊断频率。"""
        stats = self.state.record_rx_diag_event(key)
        self.state.refresh_rx_diag_rates()
        return stats

    def _should_log_error(self, key: str, min_interval_sec: float) -> bool:
        """限制同类错误日志频率。"""
        now = time.perf_counter()
        last_time = self._last_error_log_time.get(key, 0.0)
        if now - last_time < min_interval_sec:
            return False
        self._last_error_log_time[key] = now
        return True

    @staticmethod
    def _wave_label(wave_type: WaveType) -> str:
        """将波形类型转换为中文日志标签。"""
        return "信息波" if wave_type == WaveType.INFO else "干扰波"


__all__ = [
    "DecodedPacketCallback",
    "RxWorker",
]
