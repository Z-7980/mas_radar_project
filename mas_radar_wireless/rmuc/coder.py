# -*- coding: utf-8 -*-
"""RMUC2026-NC 编解码器

本模块负责 RMUC2026-NC 软件协议链路中的编解码工作，覆盖：
- CRC8 / CRC16 校验；
- 0x0A01 ~ 0x0A06 串口协议帧构造；
- 连续串口字节流滑窗解析；
- 业务数据解析与前端显示标签生成；
- 空口帧封装与从 unpacked byte 流中恢复空口帧。

重构边界：
- TX 侧仍生成 packed bytes 形式的空口帧，由 sdr_link.py 发送给 GNU Radio；
- RX 侧只接收 GNU Radio 风格的 unpacked bytes，每个 byte 表示一个判决 bit；
- 本模块不再保留旧的 bytes 级空口解码入口，也不再保留旧的 AirPacketizer；
- 空口同步规则为：Access Code 允许少量 bit 误码，Header 必须逐字节完全一致。
"""

from __future__ import annotations

from dataclasses import dataclass
import struct

from .constants import (
    ACCESS_CODES,
    AIR_FRAME_LEN,
    AIR_HEADER,
    AIR_PAYLOAD_LEN,
    CMD_FIELD_STATUS,
    CMD_INTERFERENCE_KEY,
    CMD_ROBOT_AMMO,
    CMD_ROBOT_BUFF,
    CMD_ROBOT_HP,
    CMD_ROBOT_POSITION,
    CRC8_INIT,
    CRC8_TABLE,
    CRC16_INIT,
    CRC16_TABLE,
    WaveType,
)


# ==================== CRC ====================

def crc8(data: bytes, init: int = CRC8_INIT) -> int:
    """计算 DJI/RMUC 风格帧头 CRC8。

    覆盖范围：SOF + data_length + seq，即串口帧头前 4 字节。
    """
    crc = init & 0xFF
    for byte in data:
        crc = CRC8_TABLE[(crc ^ byte) & 0xFF]
    return crc


def crc16(data: bytes, init: int = CRC16_INIT) -> int:
    """计算 DJI/RMUC 风格整帧 CRC16。

    覆盖范围：从 SOF 到 payload 结束的全部内容，不包含帧尾 CRC16 自身。
    """
    crc = init & 0xFFFF
    for byte in data:
        crc = ((crc >> 8) ^ CRC16_TABLE[(crc ^ byte) & 0xFF]) & 0xFFFF
    return crc


# ==================== 串口帧数据模型 ====================

SOF = 0xA5
"""RMUC 串口协议帧 SOF 字节。"""


@dataclass(frozen=True)
class SerialFrame:
    """已经通过 CRC 校验的 RMUC 串口协议帧。"""

    cmd_id: int
    payload: bytes
    seq: int
    raw: bytes

    @property
    def data_len(self) -> int:
        """payload 字节数，对应串口帧头中的 data_length。"""
        return len(self.payload)


@dataclass(frozen=True)
class DecodeEvent:
    """串口流解析事件。

    consumed 表示该事件从输入流前部消耗了多少字节；RX 侧用它同步空口 payload
    字节引用缓存，进而把 sof/len/seq/cmd/data/crc 标签回填到前端显示结构中。
    """

    kind: str
    consumed: int
    frame: SerialFrame | None = None
    reason: str | None = None
    cmd_id: int | None = None


# ==================== 串口帧构造 ====================

class SerialFrameBuilder:
    """串口协议帧构造器。

    TX 侧从 RuntimeState 读取业务数据后，通过本类生成 0x0A01 ~ 0x0A06 对应的
    RMUC 串口协议帧，再由 TX Worker 切入空口 payload。
    """

    def __init__(self, seq: int = 0):
        self.seq = seq & 0xFF

    def pack(self, cmd_id: int, payload: bytes) -> bytes:
        """打包通用串口帧。

        字节序约定：
        - data_length 使用小端 `<H`；
        - cmd_id 使用小端 `<H`；
        - CRC16 帧尾使用小端 `<H`；
        - SOF、seq、CRC8 为单字节，不涉及字节序。
        """
        header_without_crc8 = struct.pack("<BHB", SOF, len(payload), self.seq)
        header = header_without_crc8 + bytes([crc8(header_without_crc8)])
        body = struct.pack("<H", cmd_id) + payload
        frame_without_crc16 = header + body
        raw = frame_without_crc16 + struct.pack("<H", crc16(frame_without_crc16))
        self.seq = (self.seq + 1) & 0xFF
        return raw

    def robot_position_0x0A01(self, positions: list[tuple[int, int]]) -> bytes:
        """生成 0x0A01 机器人位置包。

        6 个机器人，每个机器人包含 X/Y 两个 int16 小端坐标，单位 cm。
        """
        if len(positions) != 6:
            raise ValueError("0x0A01 requires 6 robot positions")

        flat_values: list[int] = []
        for x_cm, y_cm in positions:
            flat_values.extend([x_cm, y_cm])
        return self.pack(CMD_ROBOT_POSITION, struct.pack("<12h", *flat_values))

    def robot_hp_0x0A02(self, hps: list[int]) -> bytes:
        """生成 0x0A02 机器人血量包。

        GUI 输入 5 个有效值：英雄、工程、步兵3、步兵4、哨兵。
        协议中第 5 个位置为保留位，因此这里自动补 0。
        """
        if len(hps) == 5:
            hps = [hps[0], hps[1], hps[2], hps[3], 0, hps[4]]
        if len(hps) != 6:
            raise ValueError("0x0A02 requires 5 values plus reserved slot, or 6 raw values")
        return self.pack(CMD_ROBOT_HP, struct.pack("<6H", *hps))

    def robot_ammo_0x0A03(self, ammo: list[int]) -> bytes:
        """生成 0x0A03 剩余发弹量包。"""
        if len(ammo) != 5:
            raise ValueError("0x0A03 requires 5 ammo values")
        return self.pack(CMD_ROBOT_AMMO, struct.pack("<5H", *ammo))

    def field_status_0x0A04(
        self,
        remain_coins: int,
        total_coins: int,
        status_flags: int,
    ) -> bytes:
        """生成 0x0A04 经济与场地状态包。

        remain/total 为 uint16 小端，status_flags 为 uint32 小端。
        """
        return self.pack(
            CMD_FIELD_STATUS,
            struct.pack("<HHI", remain_coins, total_coins, status_flags),
        )

    def robot_buff_0x0A05(
        self,
        hero: list[int],
        eng: list[int],
        inf3: list[int],
        inf4: list[int],
        sentry: list[int],
        robot_statuses: list[int],
    ) -> bytes:
        """生成 0x0A05 增益状态、哨兵姿态与机器人主要状态包。"""
        for name, values in (
            ("hero", hero),
            ("eng", eng),
            ("inf3", inf3),
            ("inf4", inf4),
        ):
            if len(values) != 5:
                raise ValueError(f"0x0A05 {name} buff requires 5 values")
        if len(sentry) != 6:
            raise ValueError("0x0A05 sentry buff requires 6 values")
        if len(robot_statuses) != 5:
            raise ValueError("0x0A05 robot statuses require 5 values")

        sentry_buff = sentry[:5]
        sentry_posture = sentry[5]
        payload = struct.pack(
            "<" + ("BHBBH" * 5) + "B" + ("B" * 5),
            *(hero + eng + inf3 + inf4 + sentry_buff + [sentry_posture] + robot_statuses),
        )
        return self.pack(CMD_ROBOT_BUFF, payload)

    def interference_key_0x0A06(self, text: str) -> bytes:
        """生成 0x0A06 干扰波密钥包。

        payload 固定为 6 字节 ASCII；不足 6 字节时右侧补字符 '0'。
        """
        payload = text[:6].encode("ascii", errors="ignore").ljust(6, b"0")
        return self.pack(CMD_INTERFERENCE_KEY, payload)


# ==================== 串口帧解析 ====================

class SerialFrameDecoder:
    """连续串口字节流滑窗解析器。

    空口 payload 中可能存在随机填充，也可能一个串口帧跨越多个空口包。
    因此 RX 侧不能假设 SOF 一定位于 payload 第 0 字节，必须使用滑窗恢复串口帧。
    """

    def __init__(self, max_payload_len: int = 200):
        self.buffer = bytearray()
        self.max_payload_len = max_payload_len

    def feed_events(self, data: bytes) -> list[DecodeEvent]:
        """喂入连续字节流，返回解析事件列表。

        返回事件包括：
        - frame：成功解析出完整串口帧；
        - error：遇到 CRC8、CRC16 或长度异常；
        - skip：跳过了非 SOF 噪声/填充字节。
        """
        self.buffer.extend(data)
        events: list[DecodeEvent] = []
        skipped = 0

        while len(self.buffer) >= 5:
            # 1. 滑窗寻找 SOF；SOF 前面的字节视为填充或噪声。
            if self.buffer[0] != SOF:
                del self.buffer[0]
                skipped += 1
                continue

            # 2. 帧头 CRC8 不通过时，丢弃当前 SOF 并继续滑窗。
            header_without_crc8 = bytes(self.buffer[:4])
            if crc8(header_without_crc8) != self.buffer[4]:
                del self.buffer[0]
                events.append(
                    DecodeEvent(kind="error", consumed=skipped + 1, reason="CRC8")
                )
                skipped = 0
                continue

            # 3. data_length 小端解析，并做最大长度保护。
            data_len = struct.unpack("<H", self.buffer[1:3])[0]
            if data_len > self.max_payload_len:
                del self.buffer[0]
                events.append(
                    DecodeEvent(kind="error", consumed=skipped + 1, reason="length")
                )
                skipped = 0
                continue

            # 4. 等待完整帧到齐：帧头 5B + cmd_id 2B + payload + CRC16 2B。
            frame_len = 5 + 2 + data_len + 2
            if len(self.buffer) < frame_len:
                break

            raw = bytes(self.buffer[:frame_len])
            expected_crc16 = struct.unpack("<H", raw[-2:])[0]

            # 5. CRC16 不通过时，丢弃当前 SOF 并继续滑窗。
            if crc16(raw[:-2]) != expected_crc16:
                cmd_id = struct.unpack("<H", raw[5:7])[0]
                del self.buffer[0]
                events.append(
                    DecodeEvent(
                        kind="error",
                        consumed=skipped + 1,
                        reason="CRC16",
                        cmd_id=cmd_id,
                    )
                )
                skipped = 0
                continue

            cmd_id = struct.unpack("<H", raw[5:7])[0]
            payload = raw[7 : 7 + data_len]
            frame = SerialFrame(cmd_id=cmd_id, payload=payload, seq=raw[3], raw=raw)

            events.append(
                DecodeEvent(
                    kind="frame",
                    consumed=skipped + frame_len,
                    frame=frame,
                    cmd_id=cmd_id,
                )
            )
            del self.buffer[:frame_len]
            skipped = 0

        if skipped:
            events.append(DecodeEvent(kind="skip", consumed=skipped))

        return events


# ==================== 业务层解析 ====================

class BusinessDecoder:
    """业务层解码器。

    将通过 CRC 的串口帧解析成 RuntimeState 和前端页面可直接使用的业务字典。
    """

    @staticmethod
    def decode(frame: SerialFrame) -> dict:
        """把串口帧解析成业务字典，并为异常包补齐对应命令的零值字段。"""
        payload = frame.payload
        payload_len = len(payload)
        command = f"0x{frame.cmd_id:04X}"

        if frame.cmd_id == CMD_ROBOT_POSITION and payload_len == 24:
            values = struct.unpack("<12h", payload)
            names = ["hero", "eng", "inf3", "inf4", "aerial", "sentry"]
            return {
                "cmd": "0x0A01",
                "payload_len": payload_len,
                "length_valid": True,
                "positions": {
                    name: {"x": values[index * 2], "y": values[index * 2 + 1]}
                    for index, name in enumerate(names)
                },
            }

        if frame.cmd_id == CMD_ROBOT_HP and payload_len == 12:
            values = struct.unpack("<6H", payload)
            return {
                "cmd": "0x0A02",
                "payload_len": payload_len,
                "length_valid": True,
                "hp": {
                    "hero": values[0],
                    "eng": values[1],
                    "inf3": values[2],
                    "inf4": values[3],
                    "sentry": values[5],
                },
            }

        if frame.cmd_id == CMD_ROBOT_AMMO and payload_len == 10:
            values = struct.unpack("<5H", payload)
            return {
                "cmd": "0x0A03",
                "payload_len": payload_len,
                "length_valid": True,
                "ammo": {
                    "hero": values[0],
                    "inf3": values[1],
                    "inf4": values[2],
                    "aerial": values[3],
                    "sentry": values[4],
                },
            }

        if frame.cmd_id == CMD_FIELD_STATUS and payload_len == 8:
            remain_coins, total_coins, status_flags = struct.unpack("<HHI", payload)
            return {
                "cmd": "0x0A04",
                "payload_len": payload_len,
                "length_valid": True,
                "field": {
                    "remain_coins": remain_coins,
                    "total_coins": total_coins,
                    "status_flags": status_flags,
                    "status_items": BusinessDecoder._decode_field_flags(status_flags),
                },
            }

        if frame.cmd_id == CMD_ROBOT_BUFF and payload_len in (36, 41):
            # 旧版 36 B 不含机器人主要状态；新版 41 B 在末尾追加 5 个状态字节。
            if payload_len == 36:
                values = struct.unpack("<" + ("BHBBH" * 4) + "BHBBHB", payload)
                robot_statuses = [0] * 5
            else:
                values = struct.unpack("<" + ("BHBBH" * 5) + "B" + ("B" * 5), payload)
                robot_statuses = values[26:31]

            names = ["hero", "eng", "inf3", "inf4", "sentry"]
            buffs = {}
            for index, name in enumerate(names):
                base = index * 5
                buffs[name] = {
                    "heal": values[base],
                    "cool": values[base + 1],
                    "def": values[base + 2],
                    "vuln": values[base + 3],
                    "atk": values[base + 4],
                }
            return {
                "cmd": "0x0A05",
                "payload_len": payload_len,
                "length_valid": True,
                "buffs": buffs,
                "sentry_posture": values[25],
                "robot_statuses": dict(zip(names, robot_statuses)),
            }

        if frame.cmd_id == CMD_INTERFERENCE_KEY and payload_len == 6:
            return {
                "cmd": "0x0A06",
                "payload_len": payload_len,
                "length_valid": True,
                "key": payload.decode("ascii", errors="replace"),
            }

        # 命令字正确但长度异常时，返回完整零值结构，避免状态更新和 ROS2 发布访问缺失键。
        decoded = BusinessDecoder._default_result(command)
        decoded.update({
            "payload_len": payload_len,
            "length_valid": False,
            "payload_hex": payload.hex().upper(),
        })
        return decoded

    @staticmethod
    def _default_result(command: str) -> dict:
        """为已知业务命令构造完整零值结果，供异常长度包安全降级。"""
        if command == "0x0A01":
            names = ["hero", "eng", "inf3", "inf4", "aerial", "sentry"]
            return {
                "cmd": command,
                "positions": {name: {"x": 0, "y": 0} for name in names},
            }

        if command == "0x0A02":
            names = ["hero", "eng", "inf3", "inf4", "sentry"]
            return {"cmd": command, "hp": {name: 0 for name in names}}

        if command == "0x0A03":
            names = ["hero", "inf3", "inf4", "aerial", "sentry"]
            return {"cmd": command, "ammo": {name: 0 for name in names}}

        if command == "0x0A04":
            return {
                "cmd": command,
                "field": {
                    "remain_coins": 0,
                    "total_coins": 0,
                    "status_flags": 0,
                    "status_items": BusinessDecoder._decode_field_flags(0),
                },
            }

        if command == "0x0A05":
            names = ["hero", "eng", "inf3", "inf4", "sentry"]
            zero_buff = {"heal": 0, "cool": 0, "def": 0, "vuln": 0, "atk": 0}
            return {
                "cmd": command,
                "buffs": {name: dict(zero_buff) for name in names},
                "sentry_posture": 0,
                "robot_statuses": {name: 0 for name in names},
            }

        if command == "0x0A06":
            return {"cmd": command, "key": "000000"}

        return {"cmd": command}

    @staticmethod
    def _decode_field_flags(flags: int) -> list[dict]:
        """按协议 bit 定义展开 0x0A04 的场地状态位。"""
        return [
            BusinessDecoder._binary_item(
                "enemy_supply_zone",
                "bit 0：对方补给区占领状态",
                flags & 0x01,
                active_color="red",
            ),
            BusinessDecoder._owner_item(
                "enemy_center_highland",
                "bit 1-2：对方中央高地的占领状态",
                (flags >> 1) & 0x03,
                allow_both=False,
            ),
            BusinessDecoder._binary_item(
                "enemy_trapezoid_highland",
                "bit 3：对方梯形高地的占领状态",
                (flags >> 3) & 0x01,
                active_color="red",
            ),
            BusinessDecoder._owner_item(
                "enemy_fortress_buff",
                "bit 4-5：对方堡垒增益点的占领状态",
                (flags >> 4) & 0x03,
                allow_both=True,
            ),
            BusinessDecoder._owner_item(
                "enemy_outpost_buff",
                "bit 6-7：对方前哨站增益点的占领状态",
                (flags >> 6) & 0x03,
                allow_both=False,
            ),
            BusinessDecoder._binary_item(
                "enemy_base_buff",
                "bit 8：对方基地增益点的占领状态",
                (flags >> 8) & 0x01,
                active_color="red",
            ),
            BusinessDecoder._detect_item(
                "enemy_fly_slope_front_tunnel",
                "bit 9：靠近对方一侧飞坡前地形跨越增益点（隧道）中心处场地交互模块卡的状态",
                (flags >> 9) & 0x01,
            ),
            BusinessDecoder._detect_item(
                "enemy_fly_slope_back_tunnel",
                "bit 10：靠近对方一侧飞坡后地形跨越增益点（隧道）中心处场地交互模块卡的状态",
                (flags >> 10) & 0x01,
            ),
            BusinessDecoder._detect_item(
                "ally_fly_slope_front_tunnel",
                "bit 11：靠近己方一侧飞坡前地形跨越增益点（隧道）中心处场地交互模块卡的状态",
                (flags >> 11) & 0x01,
            ),
            BusinessDecoder._detect_item(
                "ally_fly_slope_back_tunnel",
                "bit 12：靠近己方一侧飞坡后地形跨越增益点（隧道）中心处场地交互模块卡的状态",
                (flags >> 12) & 0x01,
            ),
            BusinessDecoder._detect_item(
                "enemy_highland_upper",
                "bit 13：对方地形跨越增益点（高地）上部场地交互模块卡的状态",
                (flags >> 13) & 0x01,
            ),
            BusinessDecoder._detect_item(
                "enemy_fly_slope_back",
                "bit 14：对方地形跨越增益点（飞坡）后部场地交互模块卡的状态",
                (flags >> 14) & 0x01,
            ),
            BusinessDecoder._detect_item(
                "enemy_highway_upper",
                "bit 15：对方地形跨越增益点（公路）上部场地交互模块卡的状态",
                (flags >> 15) & 0x01,
            ),
        ]

    @staticmethod
    def _binary_item(key: str, label: str, value: int, active_color: str) -> dict:
        """解析单 bit 占领状态项。"""
        active = bool(value)
        return {
            "key": key,
            "label": label,
            "value": int(value),
            "text": "已占领" if active else "未占领",
            "color": active_color if active else "off",
        }

    @staticmethod
    def _owner_item(key: str, label: str, value: int, allow_both: bool) -> dict:
        """解析 2 bit 所有权状态项。"""
        table = {
            0: ("未被占领", "off"),
            1: ("被对方占领", "red"),
            2: ("被己方占领", "green"),
            3: ("被双方占领" if allow_both else "保留/未知", "yellow" if allow_both else "off"),
        }
        text, color = table.get(int(value), ("未知", "off"))
        return {"key": key, "label": label, "value": int(value), "text": text, "color": color}

    @staticmethod
    def _detect_item(key: str, label: str, value: int) -> dict:
        """解析交互模块卡检测状态项。"""
        active = bool(value)
        return {
            "key": key,
            "label": label,
            "value": int(value),
            "text": "已检测到" if active else "未检测到",
            "color": "red" if active else "off",
        }


# ==================== 字节标签分类 ====================

def classify_serial_frame_bytes(frame: bytes) -> list[str]:
    """给串口帧的每个字节打来源标签，用于 GUI 颜色区分。"""
    labels: list[str] = []
    for index, _ in enumerate(frame):
        if index == 0:
            labels.append("sof")
        elif index in (1, 2):
            labels.append("len")
        elif index == 3:
            labels.append("seq")
        elif index == 4:
            labels.append("crc8")
        elif index in (5, 6):
            labels.append("cmd")
        elif index >= len(frame) - 2:
            labels.append("crc16")
        else:
            labels.append("data")
    return labels


# ==================== 空口帧数据模型 ====================

@dataclass(frozen=True)
class AirFrameRecord:
    """一次空口帧解码/封装记录。"""

    wave_type: WaveType
    frame: bytes
    payload: bytes
    payload_labels: list[str]

    def to_dict(self) -> dict:
        """转换成网页可直接渲染的字典。"""
        return {
            "wave_type": self.wave_type.value,
            "frame_hex": self.frame.hex(" ").upper(),
            "access_hex": self.frame[:8].hex(" ").upper(),
            "header_hex": self.frame[8:12].hex(" ").upper(),
            "payload": [
                {"hex": f"{byte:02X}", "label": label, "index": index}
                for index, (byte, label) in enumerate(zip(self.payload, self.payload_labels))
            ],
        }


@dataclass(frozen=True)
class AirFrameDecodeEvent:
    """空口帧诊断事件。

    当前主要用于记录：Access Code 已匹配，但 Header 不完全一致的情况。
    """

    kind: str
    wave_type: WaveType
    header_hex: str
    inverted: bool
    access_errors: int


# ==================== 空口帧封装 ====================

def pack_air_frame(wave_type: WaveType, payload_15b: bytes) -> bytes:
    """封装单个空口帧。

    字节顺序：Access Code 8B -> Header 4B -> Payload 15B。
    """
    if len(payload_15b) != AIR_PAYLOAD_LEN:
        raise ValueError("air payload must be exactly 15 bytes")
    return ACCESS_CODES[wave_type] + AIR_HEADER + payload_15b


# ==================== 空口帧解码 ====================

class AirFrameDecoder:
    """空口帧解码器。

    输入格式为 GNU Radio 解调输出的 unpacked bytes：每个 byte 表示一个判决 bit。

    解码规则：
    - 在 bit 流中滑窗寻找 Access Code；
    - Access Code 允许 max_access_errors 个 bit 误码；
    - 支持整帧极性反相检测与校正；
    - Access Code 匹配后，Header 必须与 AIR_HEADER 完全一致；
    - Header 不一致时只丢弃 1 bit 继续滑窗，避免错误同步导致整帧跳过。
    """

    def __init__(self, wave_type: WaveType, max_access_errors: int = 5):
        self.wave_type = wave_type
        self.access_code = ACCESS_CODES[wave_type]
        self.max_access_errors = int(max_access_errors)

        self.bit_buffer: list[int] = []
        self.access_bits = self._bytes_to_msb_bits(self.access_code)
        self.inverted_access_bits = [0 if bit else 1 for bit in self.access_bits]
        self.frame_bits_len = AIR_FRAME_LEN * 8

        self._events: list[AirFrameDecodeEvent] = []

    # ==================== 对外接口 ====================

    def feed_unpacked_bytes(self, unpacked_bytes: bytes) -> list[AirFrameRecord]:
        """喂入 unpacked bytes，返回恢复出的空口帧记录。"""
        if unpacked_bytes:
            self.bit_buffer.extend(byte & 0x01 for byte in unpacked_bytes)

        records: list[AirFrameRecord] = []

        while len(self.bit_buffer) >= self.frame_bits_len:
            match = self._find_access_match()
            if match is None:
                # 当前缓存中不存在一个足够长、可组成完整帧的同步位置。
                # 保留最后 frame_bits_len - 1 bit，防止 Access Code 已经出现但完整帧尚未到齐。
                keep_bits = self.frame_bits_len - 1
                if len(self.bit_buffer) > keep_bits:
                    del self.bit_buffer[:-keep_bits]
                break

            start_index, inverted, access_errors = match
            if start_index:
                del self.bit_buffer[:start_index]

            if len(self.bit_buffer) < self.frame_bits_len:
                break

            frame_bits = self.bit_buffer[: self.frame_bits_len]
            if inverted:
                frame_bits = [0 if bit else 1 for bit in frame_bits]

            frame = self._bits_to_bytes_msb(frame_bits)

            # Access Code 可以容错，但 Header 必须逐字节完全一致。
            if frame[8:12] != AIR_HEADER:
                self._events.append(
                    AirFrameDecodeEvent(
                        kind="header_error",
                        wave_type=self.wave_type,
                        header_hex=frame[8:12].hex(" ").upper(),
                        inverted=inverted,
                        access_errors=access_errors,
                    )
                )
                del self.bit_buffer[0]
                continue

            payload = frame[12:27]
            records.append(
                AirFrameRecord(
                    wave_type=self.wave_type,
                    frame=frame,
                    payload=payload,
                    payload_labels=["pad"] * AIR_PAYLOAD_LEN,
                )
            )
            del self.bit_buffer[: self.frame_bits_len]

        return records

    def pop_events(self) -> list[AirFrameDecodeEvent]:
        """返回并清空本轮空口诊断事件。"""
        events = self._events
        self._events = []
        return events

    # ==================== 内部工具 ====================

    def _find_access_match(self) -> tuple[int, bool, int] | None:
        """寻找 Access Code，允许少量误码并支持整帧反相。"""
        search_limit = len(self.bit_buffer) - self.frame_bits_len + 1
        if search_limit <= 0:
            return None

        best_match: tuple[int, bool, int] | None = None

        for index in range(search_limit):
            window = self.bit_buffer[index : index + len(self.access_bits)]

            normal_errors = self._hamming_distance(window, self.access_bits)
            if normal_errors <= self.max_access_errors:
                candidate = (index, False, normal_errors)
                best_match = self._select_better_match(best_match, candidate)
                if normal_errors == 0:
                    return candidate

            inverted_errors = self._hamming_distance(window, self.inverted_access_bits)
            if inverted_errors <= self.max_access_errors:
                candidate = (index, True, inverted_errors)
                best_match = self._select_better_match(best_match, candidate)
                if inverted_errors == 0:
                    return candidate

        return best_match

    @staticmethod
    def _select_better_match(
        current: tuple[int, bool, int] | None,
        candidate: tuple[int, bool, int],
    ) -> tuple[int, bool, int]:
        """在多个 Access Code 候选中选择更可靠者。"""
        if current is None:
            return candidate

        current_index, _current_inverted, current_errors = current
        candidate_index, _candidate_inverted, candidate_errors = candidate

        if candidate_errors < current_errors:
            return candidate
        if candidate_errors == current_errors and candidate_index < current_index:
            return candidate
        return current

    @staticmethod
    def _hamming_distance(left: list[int], right: list[int]) -> int:
        """计算两个等长 bit 序列的汉明距离。"""
        return sum(1 for left_bit, right_bit in zip(left, right) if left_bit != right_bit)

    @staticmethod
    def _bytes_to_msb_bits(data: bytes) -> list[int]:
        """将 packed bytes 按 bit7 -> bit0 顺序展开为 bit 列表。"""
        return [
            (byte >> bit_index) & 0x01
            for byte in data
            for bit_index in range(7, -1, -1)
        ]

    @staticmethod
    def _bits_to_bytes_msb(bits: list[int]) -> bytes:
        """将 MSB-first bit 列表重新合成为 packed bytes。"""
        if len(bits) % 8 != 0:
            raise ValueError("bit length must be a multiple of 8")

        output = bytearray()
        for offset in range(0, len(bits), 8):
            value = 0
            for bit in bits[offset : offset + 8]:
                value = (value << 1) | (1 if bit else 0)
            output.append(value)
        return bytes(output)


__all__ = [
    "AirFrameDecodeEvent",
    "AirFrameDecoder",
    "AirFrameRecord",
    "BusinessDecoder",
    "DecodeEvent",
    "SerialFrame",
    "SerialFrameBuilder",
    "SerialFrameDecoder",
    "classify_serial_frame_bytes",
    "crc8",
    "crc16",
    "pack_air_frame",
]
