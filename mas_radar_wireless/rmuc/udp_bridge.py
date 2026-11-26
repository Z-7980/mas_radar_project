# -*- coding: utf-8 -*-
"""RMUC2026-NC 跨机 UDP 桥接模块（B 机 / SDR 解调端）。

用途
====
在雷达"运算平台端"（A 机，mas_radar 工程）与"传感器端 / SDR 解调端"
（B 机，本 sp_rf_2026 工程）之间，建立一条**有线以太网 UDP** 数据链路。

该链路严格对应 2027《机器人制作规范》：
  - S99：运算平台端不得安装/使用无线设备 → 本链路只走有线 UDP，不涉及任何 WLAN/BT；
  - S96：运算平台端与传感器端需通过电缆连接 → 用以太网线（Cat6，建议 ≥5 m，
         直连或经自备有线交换机），与 NanoSDR 的 IIO 管理网(192.168.2.x)分网卡/分网段。

设计边界（与 ros2_bridge.py 完全对称）
====================================
本模块**不解析空口帧、不操作 ZMQ/XMLRPC**，只做两件事：
  1. 上行（B→A）：复用 SoftwareProtocolNode.add_rx_decoded_callback() 事件，
     每当一个 0x0A01~0x0A06 串口业务包 CRC 校验通过并解码成功，立即把 decoded
     结果打成 JSON 发给 A 机；并按固定周期发 status 心跳（当前解调阵营/等级、解析成功率）。
  2. 下行（A→B）：接收 A 机发来的解调配置（对方阵营 + 干扰等级），走与 ROS2 完全
     相同的入口 update_rx_selection_from_ros2() 写入 RuntimeState，并带短时保护窗口，
     避免网页周期快照抢回控制权。

失效隔离（重要）
==============
本模块被设计为"链路不可用时对宿主零影响"：
  - 构造函数内的 socket 建立/绑定失败**不会抛出**，只把 self.ok 置 False 并写运行日志；
  - self.ok 为 False 时 start() 直接返回，publish/_send 一律空操作；
  - 上行回调 publish_decoded_packet 即便发送失败也不会抛出（_send 内部兜底），
    且 rx.py 的 _emit_decoded_packet 本身已用 try/except 包裹回调，双重保险；
  - 收不到 A 机 config 时，RxSelection 仍由网页/ROS2 控制，与"没有本桥时"完全一致。
因此：网线未插 / A 机未开 / 端口冲突，都不会影响 SDR 解调、GNU Radio 流图等原功能。

线协议（JSON / UTF-8 over UDP，版本 v=1）
=======================================
通用信封::

    {"v": 1, "type": <str>, "seq": <int>, "ts": <float epoch>, ...类型字段...}

B→A（本模块发送）::

    type="packet": {"cmd": "0x0A01".."0x0A06", "data": <decoded 业务字典>}
    type="hit":    {"x": <int mm>, "y": <int mm>, "robot": <str?>, "region": <int?>}
                   # 装甲受击事件（2027 规则 5.2）。2026 解调链路暂不产生命中，
                   # 待 2027 协议明确后由解调侧调用 send_hit() 注入。
    type="status": {"rx_team": "red|blue", "rx_level": 1..3, "mode": <str>,
                    "parse_success_rate": <float>, "alive": true}

A→B（本模块接收）::

    type="config": {"team": "red|blue", "interference_level": 1..3}
    type="status": {"state": "R|B", "alive": true}     # A 机心跳，可选

端口与地址（默认在 192.168.3.0/24 业务网段，A=.10，B=.20；均可在构造函数覆盖）
==========================================================================
    A 机监听 0.0.0.0:53200（收 B 机 packet/hit/status）
    B 机监听 0.0.0.0:53201（收 A 机 config/status）
    B 机发送到 A 机 peer_host:peer_port（默认 192.168.3.10:53200）
"""

from __future__ import annotations

import json
import socket
import threading
import time


# ==================== 协议常量 ====================

PROTOCOL_VERSION = 1

DEFAULT_PEER_HOST = "192.168.3.10"
"""A 机（mas_radar）业务网口 IP。"""

DEFAULT_PEER_PORT = 53200
"""A 机监听端口，接收 B 机上行 packet/hit/status。"""

DEFAULT_LISTEN_HOST = "0.0.0.0"
DEFAULT_LISTEN_PORT = 53201
"""B 机监听端口，接收 A 机下行 config/status。"""

DEFAULT_STATUS_INTERVAL_SEC = 1.0
"""status 心跳发送周期。"""

_RECV_TIMEOUT_SEC = 0.5
"""接收 socket 超时，用于让 stop() 能及时退出 recvfrom。"""

_MAX_DATAGRAM_BYTES = 65535


# ==================== UDP 桥接器 ====================

class RadarUdpBridge:
    """把 SoftwareProtocolNode 的 RX 解包事件和控制入口桥接到跨机 UDP 链路。

    与 RadarParseEmWaveRos2Bridge 平行存在：ROS2 面向本机/同网 ROS 消费者，
    本桥面向 A 机（非 ROS 的 mas_radar）。两者可同时启用，互不影响。
    """

    def __init__(
        self,
        protocol_node,
        *,
        peer_host: str = DEFAULT_PEER_HOST,
        peer_port: int = DEFAULT_PEER_PORT,
        listen_host: str = DEFAULT_LISTEN_HOST,
        listen_port: int = DEFAULT_LISTEN_PORT,
        status_interval: float = DEFAULT_STATUS_INTERVAL_SEC,
    ):
        """创建发送/接收 socket，并把上行回调注册到协议节点。

        参数:
            protocol_node: SoftwareProtocolNode 实例（提供 add_rx_decoded_callback、
                update_rx_selection_from_ros2、state）。
            peer_host/peer_port: A 机地址，B 机上行数据发往此处。
            listen_host/listen_port: B 机本地监听地址，接收 A 机下行配置。
            status_interval: status 心跳周期（秒）。
        """
        self.protocol_node = protocol_node
        self.peer_addr = (peer_host, int(peer_port))
        self.status_interval = float(status_interval)

        self._seq = 0
        self._send_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._active = True

        self._send_sock = None
        self._recv_sock = None
        self._recv_thread: threading.Thread | None = None
        self._status_thread: threading.Thread | None = None

        # 关键：socket 建立/绑定失败绝不抛给启动入口(rmuc_node.py)，只降级为空操作。
        self.ok = True
        try:
            self._send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._recv_sock.bind((listen_host, int(listen_port)))
            self._recv_sock.settimeout(_RECV_TIMEOUT_SEC)
        except OSError as exc:
            self.ok = False
            self._send_sock = None
            self._recv_sock = None
            self._log(f"UDP bridge 初始化失败，已自动禁用（不影响解调节点原有功能）：{exc}")

        # 复用与 ROS2 桥相同的 RX 解码成功事件入口；ok=False 时 publish 会自行空操作。
        self.protocol_node.add_rx_decoded_callback(self.publish_decoded_packet)

    # ==================== 生命周期 ====================

    def start(self) -> None:
        """启动接收线程与 status 心跳线程。初始化失败时直接返回。"""
        if not self.ok:
            return
        self._recv_thread = threading.Thread(
            target=self._recv_loop, name="rmuc-udp-rx", daemon=True
        )
        self._status_thread = threading.Thread(
            target=self._status_loop, name="rmuc-udp-status", daemon=True
        )
        self._recv_thread.start()
        self._status_thread.start()
        self._log(
            f"UDP bridge started: send->{self.peer_addr}, "
            f"listen->{self._recv_sock.getsockname()}"
        )

    def stop(self) -> None:
        """停止线程并关闭 socket。"""
        self._active = False
        self._stop_event.set()
        for sock in (self._recv_sock, self._send_sock):
            if sock is None:
                continue
            try:
                sock.close()
            except OSError:
                pass

    # ==================== 上行：B -> A ====================

    def publish_decoded_packet(self, command: str, decoded: dict) -> None:
        """RX 解包成功后立即把业务包发往 A 机（由协议节点回调触发）。"""
        if not self._active or not self.ok:
            return
        self._send("packet", {"cmd": command, "data": _jsonable(decoded)})

    def send_hit(self, x: int, y: int, *, robot: str | None = None,
                 region: int | None = None) -> None:
        """上行一个装甲受击事件（2027 规则 5.2 自适应装甲加固命中流）。

        2026 解调链路暂不产生命中事件；待 2027 通信协议明确装甲受击信息的空口定义后，
        由解调侧在解析到命中坐标时调用本方法，A 机 rf_link 会转交 FortifyManager.feed_hit。
        """
        if not self._active or not self.ok:
            return
        payload: dict = {"x": int(x), "y": int(y)}
        if robot is not None:
            payload["robot"] = str(robot)
        if region is not None:
            payload["region"] = int(region)
        self._send("hit", payload)

    def _status_snapshot(self) -> dict:
        """从 RuntimeState 采集当前解调状态，供 A 机判断链路与对方干扰等级。"""
        state = self.protocol_node.state
        team, level = _split_interference_key(
            getattr(state.rx_selection, "interference_current", "red-1")
        )
        return {
            "rx_team": team,
            "rx_level": level,
            "mode": getattr(state, "mode", ""),
            "parse_success_rate": round(
                getattr(state.counters, "parse_success_rate", 1.0), 4
            ),
            "alive": True,
        }

    def _status_loop(self) -> None:
        """周期发送 status 心跳。"""
        while not self._stop_event.is_set():
            self._send("status", self._status_snapshot())
            self._stop_event.wait(self.status_interval)

    # ==================== 下行：A -> B ====================

    def _recv_loop(self) -> None:
        """接收 A 机下行数据报并分发处理。"""
        while not self._stop_event.is_set():
            try:
                raw, _addr = self._recv_sock.recvfrom(_MAX_DATAGRAM_BYTES)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                msg = json.loads(raw.decode("utf-8"))
            except (ValueError, UnicodeDecodeError):
                continue
            if not isinstance(msg, dict):
                continue
            self._handle_downlink(msg)

    def _handle_downlink(self, msg: dict) -> None:
        """处理 A 机下行消息。目前只消费 config（解调配置）。"""
        mtype = msg.get("type")
        if mtype != "config":
            return

        team = str(msg.get("team", "red")).strip().lower()
        if team not in ("red", "blue"):
            team = "red"
        try:
            level = max(1, min(3, int(msg.get("interference_level", 1))))
        except (TypeError, ValueError):
            level = 1

        interference_key = f"{team}-{level}"
        # 与 ros2_bridge._handle_demod_config 完全一致：写入 RuntimeState 并短时保护，
        # 持续收到配置时网页旧快照不会抢回控制权。
        try:
            self.protocol_node.update_rx_selection_from_ros2(
                {"interference_forced": interference_key}
            )
        except Exception as exc:  # noqa: BLE001 - 控制面异常不得中断接收线程
            self._log(f"UDP config apply failed: {exc}")

    # ==================== 底层发送 ====================

    def _send(self, mtype: str, payload: dict) -> None:
        """封装通用信封并以 UDP 发往 A 机。任何发送异常都在此吞掉，绝不影响解调节点。"""
        if not self.ok or self._send_sock is None:
            return
        envelope = {
            "v": PROTOCOL_VERSION,
            "type": mtype,
            "seq": self._next_seq(),
            "ts": time.time(),
        }
        envelope.update(payload)
        try:
            data = json.dumps(envelope, ensure_ascii=False, default=str).encode("utf-8")
            with self._send_lock:
                self._send_sock.sendto(data, self.peer_addr)
        except Exception:  # noqa: BLE001 - 网络不可达/对端离线等发送失败一律忽略
            pass

    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFFFFFFFF
        return self._seq

    def _log(self, text: str) -> None:
        try:
            self.protocol_node.state.log(text)
        except Exception:  # noqa: BLE001
            pass


# ==================== 工具函数 ====================

def _split_interference_key(key: str) -> tuple[str, int]:
    """把 "red-2" 形式的选择值拆成 (team, level)，非法输入回落 red-1。"""
    try:
        team_text, level_text = str(key).split("-", 1)
        return team_text.strip().lower(), max(1, min(3, int(level_text)))
    except (ValueError, AttributeError):
        return "red", 1


def _jsonable(obj):
    """递归保证 decoded 可 JSON 序列化；非基本类型统一转字符串，避免上行崩溃。"""
    if isinstance(obj, dict):
        return {str(k): _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v) for v in obj]
    if isinstance(obj, (str, int, float, bool)) or obj is None:
        return obj
    return str(obj)


__all__ = [
    "PROTOCOL_VERSION",
    "DEFAULT_PEER_HOST",
    "DEFAULT_PEER_PORT",
    "DEFAULT_LISTEN_PORT",
    "RadarUdpBridge",
]
