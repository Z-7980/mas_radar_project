# -*- coding: utf-8 -*-
"""RMUC2026-NC 协议软件节点协调模块

本模块负责整个协议软件节点的生命周期与模块协调。

设计边界：
- node.py 是协调层，只负责组合 TX Worker、RX Worker、SDR 数据链路和 SDR 控制器；
- TX/RX 业务状态统一保存在 RuntimeState 中；
- RX 通道选择只由网页强制指定，并写入 RuntimeState.rx_selection；
- SDR 数据面由 sdr_link.py 负责，控制面由 sdr_control.py 负责；
- node.py 不直接解析空口帧、不直接操作 ZMQ、不直接操作 XMLRPC setter。

运行模式：
- debug：TX/RX 均启用，使用 VirtualSdrLink，不启用 XMLRPC 控制面；
- loopback：TX/RX 均启用，使用 NanoSdrGnuRadioLink，并启用 XMLRPC 控制面；
- tx：仅启动 TX 线程，使用 NanoSdrGnuRadioLink，并启用 XMLRPC 控制面；
- rx：仅启动 RX 线程，使用 NanoSdrGnuRadioLink，并启用 XMLRPC 控制面。
"""

from __future__ import annotations

import threading
import time

from .rx import DecodedPacketCallback, RxWorker
from .sdr_control import SdrParameterController, SdrXmlRpcConfig
from .sdr_link import NanoSdrGnuRadioConfig, NanoSdrGnuRadioLink, SdrLink, VirtualSdrLink
from .state import RuntimeState
from .tx import TxWorker


# ==================== 模式定义 ====================

SUPPORTED_NODE_MODES = {"debug", "loopback", "tx", "rx"}
"""软件节点支持的运行模式。"""


# ==================== 协议节点总控 ====================

class SoftwareProtocolNode:
    """协议节点总控：协调 TX/RX 线程、SDR 数据链路、SDR 控制器和 GUI 更新入口。

    本类不再持有 RX 自动选择器。网页选择的 red-1 / blue-3 等解调模式会直接写入
    RuntimeState.rx_selection，再由 RuntimeState.build_sdr_params() 生成完整 SDR 控制参数。
    """

    def __init__(
        self,
        mode: str = "debug",
        rate_hz: float = 10.0,
        *,
        sdr_link: SdrLink | None = None,
        sdr_link_config: NanoSdrGnuRadioConfig | None = None,
        sdr_control_config: SdrXmlRpcConfig | None = None,
    ):
        """初始化协议节点。

        参数:
            mode: 运行模式，支持 debug / loopback / tx / rx。
            rate_hz: 软件 TX 周期生成频率，默认 10 Hz。
            sdr_link: 外部注入的数据链路实例，主要用于测试；传入后会优先使用。
            sdr_link_config: GNU Radio ZMQ 数据面配置，非 debug 模式默认需要。
            sdr_control_config: GNU Radio XMLRPC 控制面配置，非 debug 模式默认需要。
        """
        self.mode = self._normalize_mode(mode)
        self.rate_hz = float(rate_hz)

        self.state = RuntimeState(mode=self.mode)

        self.tx_enabled = self.mode in ("debug", "loopback", "tx")
        self.rx_enabled = self.mode in ("debug", "loopback", "rx")

        # SDR 数据面：debug 使用虚拟链路，真实模式使用 GNU Radio ZMQ 链路。
        self.sdr_link = sdr_link or self._build_sdr_link(self.mode, sdr_link_config)

        # SDR 控制面：debug 纯软件链路不启用 XMLRPC，避免未启动 GRC 时反复报错。
        self.sdr_controller = self._build_sdr_controller(self.mode, sdr_control_config)
        self._stop_event = threading.Event()
        self._sdr_snapshot_thread: threading.Thread | None = None
        self._ros_rx_override_until = 0.0
        """ROS2 解调配置的短时保护截止时间，避免网页周期快照立刻覆盖 ROS2 下发值。"""

        # TX Worker 始终创建，便于 rx 模式下也能复用 TxWorker.update_tx() 规范化更新 state.tx。
        # 但只有 tx_enabled=True 时才会启动 TX 后台线程。
        self.tx_worker = TxWorker(self.state, self.sdr_link, self.rate_hz)
        self.rx_worker = RxWorker(self.state, self.sdr_link) if self.rx_enabled else None

    # ==================== 生命周期 ====================

    def start(self) -> None:
        """启动当前模式需要的工作线程，并同步一次 SDR 参数。"""
        if self.sdr_controller is not None:
            self.sdr_controller.start()
            self._start_sdr_snapshot_worker()

        if self.tx_enabled:
            self.tx_worker.start()

        if self.rx_worker is not None:
            self.rx_worker.start()

        self.sync_sdr_params()
        self.state.log(f"Protocol node started: mode={self.mode}, path={self.path_name}")

    def stop(self) -> None:
        """停止所有工作线程，并关闭 SDR 数据链路。"""
        self._stop_event.set()

        self.tx_worker.stop()

        if self.rx_worker is not None:
            self.rx_worker.stop()

        if self._sdr_snapshot_thread is not None:
            self._sdr_snapshot_thread.join(timeout=2.0)

        if self.sdr_controller is not None:
            self.sdr_controller.stop()

        self.sdr_link.close()
        self.state.log("Protocol node stopped.")

    @property
    def path_name(self) -> str:
        """返回当前使用的数据链路类型描述。"""
        if isinstance(self.sdr_link, VirtualSdrLink):
            return "virtual SDR unpacked-byte queue"
        if isinstance(self.sdr_link, NanoSdrGnuRadioLink):
            return "NanoSDR/GNU Radio ZMQ"
        return type(self.sdr_link).__name__

    # ==================== GUI 更新入口 ====================

    def update_tx(
        self,
        patch: dict,
        *,
        reset_source_on_category_change: bool = True,
        clear_on_payload_change: bool = True,
    ) -> bool:
        """更新发送配置。

        该方法可同时更新 TX 控制字段和 payload 字段。真正的字段解析由 TxWorker.update_tx()
        统一完成，避免 node.py 与 tx.py 重复维护 TX patch 解析逻辑。

        参数:
            patch: 前端传入的 TX patch。
            reset_source_on_category_change: 发送源类别变化时是否清空 TX 已生成数据。
            clear_on_payload_change: payload 数据变化时是否清空 TX 已生成数据。

        返回:
            True 表示发送源类别发生变化。
        """
        source_changed = self.tx_worker.update_tx(
            patch,
            reset_source_on_category_change=reset_source_on_category_change,
            clear_on_payload_change=clear_on_payload_change,
        )

        return source_changed

    def update_rx_selection(self, patch: dict, *, source: str = "web") -> bool:
        """更新网页强制选择的 RX 解调模式。

        当前工程不再进行自动能量选择。前端只需要传入：
            {"interference_forced": "red-1" | "blue-1" | ... | "blue-3"}

        RuntimeState.rx_selection 会负责归一化 key，并同步 info_current_team。

        返回:
            True 表示实际应用的 RX 解调模式发生变化。
        """
        if source != "ros2" and time.perf_counter() < self._ros_rx_override_until:
            self.state.log("RX channel selection from GUI ignored: ROS2 override is active.")
            return False

        if "interference_forced" in patch:
            previous_key = self.state.rx_selection.interference_current
            self.state.rx_selection.apply_forced_interference(patch.get("interference_forced"))
            current_key = self.state.rx_selection.interference_current
            if previous_key != current_key:
                self.state.log(f"RX channel selection updated: {current_key}")
                return True

        return False

    def update_rx_selection_from_ros2(self, patch: dict, *, hold_sec: float = 2.0) -> bool:
        """应用 ROS2 解调参数配置，并短时保护该配置不被网页周期快照覆盖。

        网页会每 5 秒上传一次完整 SDR 快照，其中可能包含用户界面里的旧 RX 选择。
        ROS2 是部署时的上位机入口，因此收到 ROS2 消息后先写入 RuntimeState，并设置
        一个短时保护窗口；持续发布 ROS2 配置时，网页旧快照不会抢回控制权。
        """
        self._ros_rx_override_until = time.perf_counter() + max(0.0, float(hold_sec))
        return self.update_rx_selection(patch, source="ros2")

    def update_sdr_control(self, patch: dict) -> None:
        """一次性应用网页 SDR 控制快照。

        前端 /api/sdr 会同时上传 TX 控制快照与 RX 解调选择：
            {
                "tx": {...},
                "rx": {"interference_forced": "red-3"}
            }

        本方法先更新 state，再执行一次 SDR 参数同步，避免 TX/RX 分散更新导致硬件短时间
        进入中间状态。
        """
        tx_patch = dict(patch.get("tx", {}) or {})
        rx_patch = dict(patch.get("rx", {}) or {})

        source_changed = self.update_tx(
            tx_patch,
            reset_source_on_category_change=True,
            clear_on_payload_change=False,
        )
        self.update_rx_selection(rx_patch)

        if source_changed:
            self.state.log("TX source category changed; generated TX data was reset before SDR sync.")

        self.sync_sdr_params()

    def add_rx_decoded_callback(self, callback: DecodedPacketCallback) -> None:
        """注册 RX 业务包解码成功回调。

        ROS2 bridge 通过该入口订阅内部解码事件。该事件只在串口帧 CRC 校验通过后触发，
        不影响非 ROS 调试模式，也不改变现有网页 GUI 的状态来源。
        """
        if self.rx_worker is None:
            self.state.log("RX decoded callback ignored: RX worker is disabled in this mode.")
            return

        self.rx_worker.add_decoded_callback(callback)

    # ==================== SDR 参数同步 ====================

    def sync_sdr_params(self, *, force: bool = False) -> bool:
        """将 RuntimeState 中的 SDR 参数快照加入发送队列。

        返回:
            True 表示本次实际执行了 XMLRPC 同步；False 表示控制器未启用、参数未变化，
            或同步失败后已记录日志。
        """
        if self.sdr_controller is None:
            return False

        try:
            synced = self.sdr_controller.enqueue_from_state(self.state, force=force)
        except Exception as exc:
            self.state.log(f"SDR parameter enqueue failed: {exc}")
            return False

        if synced:
            self.state.log("SDR parameter snapshot queued.")
        return bool(synced)

    def _start_sdr_snapshot_worker(self) -> None:
        """启动 SDR 控制快照轮询线程。"""
        if self._sdr_snapshot_thread is not None and self._sdr_snapshot_thread.is_alive():
            return

        self._sdr_snapshot_thread = threading.Thread(
            target=self._sdr_snapshot_worker,
            name="sdr-control-snapshot",
            daemon=True,
        )
        self._sdr_snapshot_thread.start()

    def _sdr_snapshot_worker(self) -> None:
        """每 5 s 获取一次网页状态对应的 SDR 控制快照并入队。"""
        while not self._stop_event.is_set():
            self.sync_sdr_params()
            self._stop_event.wait(5.0)

    # ==================== 构建工具 ====================

    @staticmethod
    def _normalize_mode(mode: str) -> str:
        """校验并归一化运行模式。"""
        normalized = str(mode).strip().lower()
        if normalized not in SUPPORTED_NODE_MODES:
            choices = ", ".join(sorted(SUPPORTED_NODE_MODES))
            raise ValueError(f"unknown node mode: {mode!r}; expected one of: {choices}")
        return normalized

    @staticmethod
    def _build_sdr_link(
        mode: str,
        link_config: NanoSdrGnuRadioConfig | None,
    ) -> SdrLink:
        """根据运行模式构建 SDR 数据面链路。"""
        if mode == "debug":
            return VirtualSdrLink()

        if link_config is None:
            raise ValueError("non-debug mode requires sdr_link_config")

        return NanoSdrGnuRadioLink(link_config)

    @staticmethod
    def _build_sdr_controller(
        mode: str,
        control_config: SdrXmlRpcConfig | None,
    ) -> SdrParameterController | None:
        """根据运行模式构建 SDR XMLRPC 控制器。"""
        if mode == "debug":
            return None

        if control_config is None:
            raise ValueError("non-debug mode requires sdr_control_config")

        return SdrParameterController(control_config)


__all__ = [
    "DecodedPacketCallback",
    "SUPPORTED_NODE_MODES",
    "SoftwareProtocolNode",
]
