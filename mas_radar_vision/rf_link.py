# -*- coding: utf-8 -*-
"""雷达跨机 UDP 链路客户端（A 机 / 运算平台端，mas_radar_temp）。

用途
====
与 B 机（SDR 解调端，sp_rf_2026 工程 rmuc/udp_bridge.py）建立**有线以太网 UDP** 链路，
把 B 机解调出的信息波数据接入本机决策逻辑，并把解调配置下发给 B 机。
对应 2027《机器人制作规范》S99（运算端禁无线→纯有线 UDP）与 S96（两端电缆连接，≥5m）。

与本版 mas_radar_temp 的三个集成点
================================
  1. opp_jam_level（main.py 全局，压制门控用）      ← status.rx_level 经 on_jam_level 注入
  2. fortify_logic.FortifyManager.feed_hit(x,y,zone) ← hit 消息注入（本版签名含区域类别）
  3. last_pw_cmd / last_pw_bytes（main.py 密钥暂存） ← 0x0A06 密钥经 on_secret_key 注入

失效隔离（与 B 机 udp_bridge.py 同口径）
======================================
  - 构造内 socket 建立/绑定失败不抛出，置 self.ok=False 并告警；
  - ok=False 时 start() 直接返回；所有发送/回调内部吞异常；
  - 断链时 opp_jam_level 保持 None、命中流保持为空、密钥保持 b'000000'，
    等价于"没有 B 机时"的原有行为。网线未插/对端未开/端口冲突均不影响雷达原功能。

线协议与 sp_rf_2026/rmuc/udp_bridge.py 完全一致（JSON / UTF-8 over UDP，v=1）。
默认网段 192.168.3.0/24：A 机=.10，B 机=.20。
  A 机监听 0.0.0.0:53200；B 机监听 0.0.0.0:53201；A 机发往 B 机 53201。
"""

from __future__ import annotations

import json
import socket
import threading
import time
from collections import deque


# ==================== 协议常量（须与 B 机 udp_bridge.py 保持一致） ====================

PROTOCOL_VERSION = 1

DEFAULT_PEER_HOST = "192.168.3.20"
DEFAULT_PEER_PORT = 53201
DEFAULT_LISTEN_HOST = "0.0.0.0"
DEFAULT_LISTEN_PORT = 53200
DEFAULT_CONFIG_INTERVAL_SEC = 1.0
_RECV_TIMEOUT_SEC = 0.5
_LINK_TIMEOUT_SEC = 3.0
_MAX_DATAGRAM_BYTES = 65535


# ==================== UDP 链路客户端 ====================

class RfLinkClient:
    """A 机侧跨机 UDP 链路客户端（收信息波数据 + 发解调配置）。

    仅依赖标准库；与 main.py 的耦合全部经构造参数注入（fortify_mgr、回调），
    任何内部异常不外溢到宿主。
    """

    def __init__(
        self,
        *,
        own_team: str,
        fortify_mgr=None,
        peer_host: str = DEFAULT_PEER_HOST,
        peer_port: int = DEFAULT_PEER_PORT,
        listen_host: str = DEFAULT_LISTEN_HOST,
        listen_port: int = DEFAULT_LISTEN_PORT,
        config_interval: float = DEFAULT_CONFIG_INTERVAL_SEC,
        on_jam_level=None,
        on_secret_key=None,
        on_packet=None,
    ):
        """
        参数:
            own_team: 本机阵营 'R'/'B'（main.py 的 state），对方阵营自动推导。
            fortify_mgr: FortifyManager 实例；hit 事件注入其 feed_hit(x, y, zone)。
            peer_host/peer_port: B 机地址。
            listen_host/listen_port: A 机本地监听地址。
            config_interval: config 下发周期（秒）。
            on_jam_level: 回调 fn(level:int)，收到对方干扰难度时触发（更新 opp_jam_level）。
            on_secret_key: 回调 fn(key:str)，解出 0x0A06 密钥时触发（更新 last_pw_*）。
            on_packet: 回调 fn(cmd:str, data:dict)，可选，消费每个 decoded 包。
        """
        self.own_team = str(own_team).strip().upper()
        self.opp_team = "red" if self.own_team == "B" else "blue"
        self.fortify_mgr = fortify_mgr
        self.on_jam_level = on_jam_level
        self.on_secret_key = on_secret_key
        self.on_packet = on_packet

        self.peer_addr = (peer_host, int(peer_port))
        self.config_interval = float(config_interval)

        # ===== 与上层共享的运行数据（整体赋值，读取受 GIL 保护） =====
        self.rf_info: dict = {}          # cmd -> 最新一次 decoded 业务字典
        self.opp_secret_key = None       # 0x0A06 解出的对方干扰波密钥
        self.opp_jam_level = None        # B 机上报的对方干扰波难度（1~3）
        self.link_alive = False          # B 机心跳是否活跃
        self.target_level = 1            # 要求 B 机解调的干扰等级（密钥上报成功后再抬升）

        # ===== UI 展示辅助状态（rf_ui_panel.py 消费，不影响协议与主流程） =====
        self.rf_updated_at: dict = {}    # cmd -> 最近一次解码上行的本地时间戳
        self.rx_packet_count = 0
        self.rx_hit_count = 0
        self.rx_status_count = 0
        self.last_hit_ts = None
        self.last_config_sent = None     # 最近一次 config 下发时刻（本地时钟）
        self.recent_hits = deque(maxlen=8)  # 最近 8 条受击事件 (x, y, zone, ts)

        self._last_status_ts = 0.0
        self._seq = 0
        self._send_lock = threading.Lock()
        self._stop_event = threading.Event()

        self._send_sock = None
        self._recv_sock = None
        self._recv_thread: threading.Thread | None = None
        self._config_thread: threading.Thread | None = None

        # 关键：socket 建立/绑定失败绝不抛给宿主(main.py)，只降级为空操作。
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
            print(f"[rf_link] UDP 链路初始化失败，已自动禁用（不影响雷达原有功能）：{exc}")

    # ==================== 生命周期 ====================

    def start(self) -> None:
        """启动接收线程与 config 下发线程（daemon）。初始化失败时直接返回。"""
        if not self.ok:
            return
        self._recv_thread = threading.Thread(
            target=self._recv_loop, name="rf-link-rx", daemon=True
        )
        self._config_thread = threading.Thread(
            target=self._config_loop, name="rf-link-cfg", daemon=True
        )
        self._recv_thread.start()
        self._config_thread.start()

    def stop(self) -> None:
        """停止线程并关闭 socket。"""
        self._stop_event.set()
        for sock in (self._recv_sock, self._send_sock):
            if sock is None:
                continue
            try:
                sock.close()
            except OSError:
                pass

    # ==================== 下行：A -> B ====================

    def _config_loop(self) -> None:
        """周期下发解调配置（对方阵营 + 目标干扰等级）。"""
        while not self._stop_event.is_set():
            self._send(
                "config",
                {"team": self.opp_team, "interference_level": int(self.target_level)},
            )
            self.last_config_sent = time.time()
            self._stop_event.wait(self.config_interval)

    def set_target_level(self, level: int) -> None:
        """设置要求 B 机解调的干扰等级（1~3）。

        典型用法：A 机将对方密钥成功上报裁判服务器、使对方干扰难度提升后，
        调用本方法跟随切换 B 机解调频段。
        """
        self.target_level = max(1, min(3, int(level)))

    # ==================== 上行：B -> A ====================

    def _recv_loop(self) -> None:
        """接收 B 机上行数据报并分发。"""
        while not self._stop_event.is_set():
            try:
                raw, _addr = self._recv_sock.recvfrom(_MAX_DATAGRAM_BYTES)
            except socket.timeout:
                self._check_link_timeout()
                continue
            except OSError:
                break
            try:
                msg = json.loads(raw.decode("utf-8"))
            except (ValueError, UnicodeDecodeError):
                continue
            if not isinstance(msg, dict):
                continue
            self._handle_uplink(msg)

    def _handle_uplink(self, msg: dict) -> None:
        """按 type 分发 B 机上行消息。所有分支均兜底，异常不外溢。"""
        mtype = msg.get("type")

        if mtype == "packet":
            cmd = msg.get("cmd")
            data = msg.get("data") or {}
            if cmd:
                self.rf_info[cmd] = data
                self.rf_updated_at[cmd] = time.time()
                self.rx_packet_count += 1
                if cmd == "0x0A06":
                    key = data.get("key")
                    if key:
                        self.opp_secret_key = str(key)
                        if self.on_secret_key is not None:
                            try:
                                self.on_secret_key(str(key))
                            except Exception:  # noqa: BLE001
                                pass
                if self.on_packet is not None:
                    try:
                        self.on_packet(cmd, data)
                    except Exception:  # noqa: BLE001
                        pass

        elif mtype == "hit":
            # 2027 规则 5.2：装甲受击坐标（检测面局部系）注入加固决策器。
            # 本版 fortify_logic.feed_hit 签名为 (x, y, zone)，zone=命中区域类别。
            # 统计与 recent_hits 不依赖 fortify_mgr，独立监控壳(rf_link_monitor)也能看到。
            try:
                x = int(msg.get("x", 0))
                y = int(msg.get("y", 0))
                zone = int(msg.get("region", msg.get("zone", 0)))
                self.rx_hit_count += 1
                self.last_hit_ts = time.time()
                self.recent_hits.append((x, y, zone, self.last_hit_ts))
                if self.fortify_mgr is not None:
                    self.fortify_mgr.feed_hit(x, y, zone)
            except Exception:  # noqa: BLE001
                pass

        elif mtype == "status":
            self.link_alive = True
            self.rx_status_count += 1
            self._last_status_ts = time.time()
            level = msg.get("rx_level")
            try:
                level = None if level is None else int(level)
            except (TypeError, ValueError):
                level = None
            if level is not None:
                self.opp_jam_level = level
                if self.on_jam_level is not None:
                    try:
                        self.on_jam_level(level)
                    except Exception:  # noqa: BLE001
                        pass

    def _check_link_timeout(self) -> None:
        """超时未收到心跳则标记链路断开。"""
        if self.link_alive and (time.time() - self._last_status_ts) > _LINK_TIMEOUT_SEC:
            self.link_alive = False

    # ==================== UI 快照（rf_ui_panel.py 消费） ====================

    def snapshot(self) -> dict:
        """返回供界面绘制/日志的只读快照；全部纯标准类型，任何情况下不抛异常。"""
        try:
            now = time.time()
            return {
                "ok": self.ok,
                "link_alive": self.link_alive,
                "peer": self.peer_addr,
                "own_team": self.own_team,
                "opp_team": self.opp_team,
                "target_level": self.target_level,
                "opp_jam_level": self.opp_jam_level,
                "opp_secret_key": self.opp_secret_key,
                "status_age_sec": None if self._last_status_ts <= 0 else round(now - self._last_status_ts, 1),
                "config_beat_sec": None if self.last_config_sent is None else round(now - self.last_config_sent, 1),
                "counts": {
                    "packet": self.rx_packet_count,
                    "hit": self.rx_hit_count,
                    "status": self.rx_status_count,
                },
                "rf_info": dict(self.rf_info),
                "rf_updated_at": dict(self.rf_updated_at),
                "recent_hits": list(self.recent_hits),
            }
        except Exception:  # noqa: BLE001 - 快照失败界面显示空，绝不影响主流程
            return {}

    # ==================== 底层发送 ====================

    def _send(self, mtype: str, payload: dict) -> None:
        """封装通用信封以 UDP 发往 B 机。任何发送异常均在此吞掉，绝不影响宿主。"""
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


__all__ = [
    "PROTOCOL_VERSION",
    "DEFAULT_PEER_HOST",
    "DEFAULT_PEER_PORT",
    "DEFAULT_LISTEN_PORT",
    "RfLinkClient",
]
