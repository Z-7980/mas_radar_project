# -*- coding: utf-8 -*-
"""RMUC2026-NC ROS2 桥接模块。

本模块把当前工程内部的 RuntimeState / RX 解包事件映射到 ROS2：
- 订阅解调参数配置，并以 ROS2 配置为最高优先级更新网页状态和 SDR 参数；
- 每当某个 0x0A01~0x0A06 串口业务包 CRC 校验通过并成功解码后，立即发布对应 topic。
"""

from __future__ import annotations

import threading
import time

from rclpy.node import Node

from radar_parse_em_wave.msg import (
    RadarParseEmWave0A01RobotPosition,
    RadarParseEmWave0A02RobotHp,
    RadarParseEmWave0A03RobotAmmo,
    RadarParseEmWave0A04FieldStatus,
    RadarParseEmWave0A05RobotBuff,
    RadarParseEmWave0A06InterferenceKey,
    RadarParseEmWaveDemodConfig,
)

from .constants import Team
from .node import SoftwareProtocolNode


# ==================== ROS2 话题配置 ====================

DEMOD_CONFIG_TOPIC = "/rmuc/demod_config"
"""解调参数配置订阅话题。"""

RX_PACKET_TOPICS = {
    "0x0A01": "/rmuc/rx/packet_0x0a01",
    "0x0A02": "/rmuc/rx/packet_0x0a02",
    "0x0A03": "/rmuc/rx/packet_0x0a03",
    "0x0A04": "/rmuc/rx/packet_0x0a04",
    "0x0A05": "/rmuc/rx/packet_0x0a05",
    "0x0A06": "/rmuc/rx/packet_0x0a06",
}
"""业务包发布话题。topic 片段不能以数字开头，因此加 packet_ 前缀。"""

ROS2_DEMOD_FORCE_SYNC_DEADZONE_SEC = 30.0
"""ROS2 解调配置触发兜底强制同步 SDR 的最小间隔，单位秒。"""


# ==================== ROS2 桥接器 ====================

class RadarParseEmWaveRos2Bridge:
    """把 SoftwareProtocolNode 的 RX 解包事件和控制入口桥接到 ROS2。"""

    def __init__(self, protocol_node: SoftwareProtocolNode):
        """创建 ROS2 节点、发布器、订阅器，并注册 RX 解包回调。"""
        self.protocol_node = protocol_node
        self.ros_node = Node("radar_parse_em_wave_node")
        self._publish_lock = threading.Lock()
        self._active = True
        self._last_forced_demod_sync_at = 0.0

        self._demod_sub = self.ros_node.create_subscription(
            RadarParseEmWaveDemodConfig,
            DEMOD_CONFIG_TOPIC,
            self._handle_demod_config,
            10,
        )

        self._packet_publishers = {
            "0x0A01": self.ros_node.create_publisher(
                RadarParseEmWave0A01RobotPosition,
                RX_PACKET_TOPICS["0x0A01"],
                10,
            ),
            "0x0A02": self.ros_node.create_publisher(
                RadarParseEmWave0A02RobotHp,
                RX_PACKET_TOPICS["0x0A02"],
                10,
            ),
            "0x0A03": self.ros_node.create_publisher(
                RadarParseEmWave0A03RobotAmmo,
                RX_PACKET_TOPICS["0x0A03"],
                10,
            ),
            "0x0A04": self.ros_node.create_publisher(
                RadarParseEmWave0A04FieldStatus,
                RX_PACKET_TOPICS["0x0A04"],
                10,
            ),
            "0x0A05": self.ros_node.create_publisher(
                RadarParseEmWave0A05RobotBuff,
                RX_PACKET_TOPICS["0x0A05"],
                10,
            ),
            "0x0A06": self.ros_node.create_publisher(
                RadarParseEmWave0A06InterferenceKey,
                RX_PACKET_TOPICS["0x0A06"],
                10,
            ),
        }

        self.protocol_node.add_rx_decoded_callback(self.publish_decoded_packet)

    # ==================== 生命周期 ====================

    def destroy(self) -> None:
        """销毁 ROS2 节点，后续 RX 回调将不再发布消息。"""
        self._active = False
        self.ros_node.destroy_node()

    # ==================== 订阅处理 ====================

    def _handle_demod_config(self, msg: RadarParseEmWaveDemodConfig) -> None:
        """处理 ROS2 下发的解调参数配置。

        ROS2 配置优先级高于网页 GUI。这里直接写入 RuntimeState.rx_selection，
        网页下一次轮询 /api/state 时会显示 ROS2 强制后的阵营和干扰等级。
        """
        team = self._normalize_team(msg.team)
        level = max(1, min(3, int(msg.interference_level)))
        interference_key = f"{team.value}-{level}"
        now = time.perf_counter()
        before_snapshot = self.protocol_node.state.build_sdr_params()

        self.protocol_node.update_rx_selection_from_ros2(
            {"interference_forced": interference_key}
        )
        after_snapshot = self.protocol_node.state.build_sdr_params()
        snapshot_changed = before_snapshot != after_snapshot
        should_force_sync = (
            snapshot_changed
            or now - self._last_forced_demod_sync_at >= ROS2_DEMOD_FORCE_SYNC_DEADZONE_SEC
        )

        if should_force_sync:
            synced = self.protocol_node.sync_sdr_params(force=True)
            if synced:
                self._last_forced_demod_sync_at = now
                self.ros_node.get_logger().info(
                    f"demod config force-synced: {interference_key}, "
                    f"snapshot_changed={snapshot_changed}"
                )
                self.protocol_node.state.log(
                    f"ROS2 解调参数配置已强制同步到 SDR: {interference_key}"
                )
        else:
            self.ros_node.get_logger().debug(
                f"demod config snapshot unchanged in deadzone: {interference_key}"
            )

    @staticmethod
    def _normalize_team(team_text: str) -> Team:
        """将 ROS2 消息中的阵营字段归一化为 Team 枚举。"""
        normalized = str(team_text or "").strip().lower()
        if normalized in ("blue", "b", "1"):
            return Team.BLUE
        return Team.RED

    # ==================== 发布处理 ====================

    def publish_decoded_packet(self, command: str, decoded: dict) -> None:
        """RX 解包成功后立即发布对应 ROS2 topic。"""
        if not self._active:
            return

        publisher = self._packet_publishers.get(command)
        if publisher is None:
            return

        msg = self._build_packet_message(command, decoded)
        if msg is None:
            return

        with self._publish_lock:
            publisher.publish(msg)

    def _build_packet_message(self, command: str, decoded: dict):
        """根据业务包 ID 构造对应的 ROS2 消息。"""
        if command == "0x0A01":
            msg = RadarParseEmWave0A01RobotPosition()
            self._fill_header(msg, "rmuc_0a01")
            positions = decoded.get("positions", {})
            self._fill_position_msg(msg, positions)
            return msg

        if command == "0x0A02":
            msg = RadarParseEmWave0A02RobotHp()
            self._fill_header(msg, "rmuc_0a02")
            hp = decoded.get("hp", {})
            msg.hero_hp = self._uint16(hp.get("hero"))
            msg.engineer_hp = self._uint16(hp.get("eng"))
            msg.infantry3_hp = self._uint16(hp.get("inf3"))
            msg.infantry4_hp = self._uint16(hp.get("inf4"))
            msg.sentry_hp = self._uint16(hp.get("sentry"))
            return msg

        if command == "0x0A03":
            msg = RadarParseEmWave0A03RobotAmmo()
            self._fill_header(msg, "rmuc_0a03")
            ammo = decoded.get("ammo", {})
            msg.hero_ammo = self._uint16(ammo.get("hero"))
            msg.infantry3_ammo = self._uint16(ammo.get("inf3"))
            msg.infantry4_ammo = self._uint16(ammo.get("inf4"))
            msg.aerial_ammo = self._uint16(ammo.get("aerial"))
            msg.sentry_ammo = self._uint16(ammo.get("sentry"))
            return msg

        if command == "0x0A04":
            msg = RadarParseEmWave0A04FieldStatus()
            self._fill_header(msg, "rmuc_0a04")
            field = decoded.get("field", {})
            msg.remain_coins = self._uint16(field.get("remain_coins"))
            msg.total_coins = self._uint16(field.get("total_coins"))
            msg.status_flags = self._uint32(field.get("status_flags"))
            return msg

        if command == "0x0A05":
            msg = RadarParseEmWave0A05RobotBuff()
            self._fill_header(msg, "rmuc_0a05")
            self._fill_buff_msg(msg, decoded)
            return msg

        if command == "0x0A06":
            msg = RadarParseEmWave0A06InterferenceKey()
            self._fill_header(msg, "rmuc_0a06")
            msg.key = str(decoded.get("key", ""))
            return msg

        return None

    def _fill_header(self, msg, frame_id: str) -> None:
        """填充消息头时间戳和 frame_id。"""
        msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

    @staticmethod
    def _fill_position_msg(msg: RadarParseEmWave0A01RobotPosition, positions: dict) -> None:
        """填充 0x0A01 机器人位置消息。"""
        mapping = {
            "hero": ("hero_x", "hero_y"),
            "eng": ("engineer_x", "engineer_y"),
            "inf3": ("infantry3_x", "infantry3_y"),
            "inf4": ("infantry4_x", "infantry4_y"),
            "aerial": ("aerial_x", "aerial_y"),
            "sentry": ("sentry_x", "sentry_y"),
        }
        for robot_name, (x_attr, y_attr) in mapping.items():
            position = positions.get(robot_name, {})
            setattr(msg, x_attr, int(position.get("x", 0)))
            setattr(msg, y_attr, int(position.get("y", 0)))

    def _fill_buff_msg(self, msg: RadarParseEmWave0A05RobotBuff, decoded: dict) -> None:
        """填充 V2 版 0x0A05 增益、哨兵姿态和主要状态消息。"""
        buffs = decoded.get("buffs", {})
        prefix_map = {
            "hero": "hero",
            "eng": "engineer",
            "inf3": "infantry3",
            "inf4": "infantry4",
            "sentry": "sentry",
        }
        for robot_name, prefix in prefix_map.items():
            buff = buffs.get(robot_name, {})
            setattr(msg, f"{prefix}_heal", self._uint8(buff.get("heal")))
            setattr(msg, f"{prefix}_cool", self._uint16(buff.get("cool")))
            setattr(msg, f"{prefix}_def", self._uint8(buff.get("def")))
            setattr(msg, f"{prefix}_vuln", self._uint8(buff.get("vuln")))
            setattr(msg, f"{prefix}_atk", self._uint16(buff.get("atk")))

        statuses = decoded.get("robot_statuses", {})
        msg.sentry_posture = self._uint8(decoded.get("sentry_posture"))
        msg.hero_status = self._uint8(statuses.get("hero"))
        msg.engineer_status = self._uint8(statuses.get("eng"))
        msg.infantry3_status = self._uint8(statuses.get("inf3"))
        msg.infantry4_status = self._uint8(statuses.get("inf4"))
        msg.sentry_status = self._uint8(statuses.get("sentry"))

    @staticmethod
    def _uint8(value) -> int:
        """转换为 uint8 范围整数。"""
        return max(0, min(0xFF, int(value or 0)))

    @staticmethod
    def _uint16(value) -> int:
        """转换为 uint16 范围整数。"""
        return max(0, min(0xFFFF, int(value or 0)))

    @staticmethod
    def _uint32(value) -> int:
        """转换为 uint32 范围整数。"""
        return max(0, min(0xFFFFFFFF, int(value or 0)))


__all__ = [
    "DEMOD_CONFIG_TOPIC",
    "RX_PACKET_TOPICS",
    "ROS2_DEMOD_FORCE_SYNC_DEADZONE_SEC",
    "RadarParseEmWaveRos2Bridge",
]
