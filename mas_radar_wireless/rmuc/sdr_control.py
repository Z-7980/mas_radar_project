# -*- coding: utf-8 -*-
"""RMUC2026-NC SDR 控制面。

本模块只负责 GNU Radio XMLRPC 离散参数设置和 RX gain 自适应控制，不处理 ZMQ bit 数据流。
控制策略：
- RuntimeState 周期性生成完整 SDR 参数快照；
- 参数快照发生变化时进入发送队列；
- 队列后台线程每 0.5 s 发送一条快照，避免短时间连续调用过多 XMLRPC setter；
- RX gain 自适应控制也只向同一个队列追加快照，保证所有硬件控制量按序下发。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import queue
import struct
import threading
import time
from typing import TYPE_CHECKING, ClassVar, Mapping
from xmlrpc.client import ServerProxy

if TYPE_CHECKING:
    from .state import RuntimeState


# ==================== 类型定义 ====================

SdrParameterValue = int | float
SdrParameterSnapshot = dict[str, SdrParameterValue]


# ==================== 控制默认值 ====================

SDR_QUEUE_SEND_INTERVAL_SEC = 0.5
"""发送队列中相邻两条参数快照的最小发送间隔，单位秒。"""

SDR_PARAMETER_QUEUE_MAXSIZE = 64
"""SDR 参数发送队列最大长度，避免控制端积压无限增长。"""

RX_GAIN_MIN = 0
RX_GAIN_MAX = 60
RX_GAIN_DEFAULT = 40
RX_GAIN_UPDATE_DEADZONE_SEC = 5.0
RX_AVG_TARGET_DB = -15.0
RX_AVG_DEADZONE_LOW_DB = -20.0
RX_AVG_DEADZONE_HIGH_DB = -10.0


# ==================== XMLRPC / telemetry 配置 ====================

@dataclass(frozen=True)
class SdrXmlRpcConfig:
    """GNU Radio 控制面配置。

    host/port 用于 XMLRPC setter；telemetry_host/telemetry_port 用于接收流图输出的
    RX 功率特征。telemetry 只影响自适应 RX gain，不参与 bit 数据收发。
    """

    host: str
    port: int
    telemetry_host: str | None = None
    telemetry_port: int | None = None
    queue_send_interval_sec: float = SDR_QUEUE_SEND_INTERVAL_SEC


# ==================== SDR 参数控制器 ====================

class SdrParameterController:
    """SDR 参数控制器：维护发送队列并执行 XMLRPC 下发。"""

    TX_PARAMETER_NAMES: ClassVar[tuple[str, ...]] = (
        "tx_center_freq1",
        "tx_center_freq2",
        "tx_attenuation1",
        "tx_attenuation2",
        "tx_mod_info_en",
        "tx_mod_int1_en",
        "tx_mod_int2_en",
        "tx_mod_int3_en",
    )
    """TX 侧同步参数列表。"""

    RX_PARAMETER_NAMES: ClassVar[tuple[str, ...]] = (
        "rx_center_freq",
        "rx_interfere_channel_freq",
        "rx_info_channel_freq",
        "rx_int_bw",
    )
    """RX 侧同步参数列表。"""

    SDR_PARAMETER_NAMES: ClassVar[tuple[str, ...]] = TX_PARAMETER_NAMES + RX_PARAMETER_NAMES
    """完整 SDR 参数列表，字段名必须与 GNU Radio 变量 setter 匹配。"""

    def __init__(self, xmlrpc_config: SdrXmlRpcConfig):
        """初始化控制器和后台线程状态。"""
        self.xmlrpc_config = xmlrpc_config
        self._parameter_queue: queue.Queue[SdrParameterSnapshot] = queue.Queue(
            maxsize=SDR_PARAMETER_QUEUE_MAXSIZE,
        )

        self._queue_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._sender_thread: threading.Thread | None = None
        self._telemetry_thread: threading.Thread | None = None

        self._last_enqueued_control_snapshot: SdrParameterSnapshot | None = None
        self._last_enqueued_gain_snapshot: SdrParameterSnapshot | None = None
        self._last_sent_snapshot: SdrParameterSnapshot | None = None
        self._runtime_state: RuntimeState | None = None

        self._current_rx_gain = RX_GAIN_DEFAULT
        self._last_rx_gain_update_time = 0.0

    # ==================== 生命周期 ====================

    def start(self) -> None:
        """启动参数发送线程和可选 telemetry 接收线程。"""
        if self._sender_thread is None or not self._sender_thread.is_alive():
            self._sender_thread = threading.Thread(
                target=self._send_queue_worker,
                name="sdr-parameter-sender",
                daemon=True,
            )
            self._sender_thread.start()

        if (
            self.xmlrpc_config.telemetry_host
            and self.xmlrpc_config.telemetry_port is not None
            and (self._telemetry_thread is None or not self._telemetry_thread.is_alive())
        ):
            self._telemetry_thread = threading.Thread(
                target=self._telemetry_worker,
                name="sdr-rx-power-telemetry",
                daemon=True,
            )
            self._telemetry_thread.start()

    def stop(self) -> None:
        """停止后台线程。"""
        self._stop_event.set()

        if self._sender_thread is not None:
            self._sender_thread.join(timeout=2.0)
        if self._telemetry_thread is not None:
            self._telemetry_thread.join(timeout=2.0)

    # ==================== 对外入队接口 ====================

    def enqueue_from_state(self, runtime_state: "RuntimeState", *, force: bool = False) -> bool:
        """从 RuntimeState 生成完整参数快照并入队。"""
        self._runtime_state = runtime_state
        self._current_rx_gain = int(runtime_state.rx_gain.current_gain)
        return self.enqueue_parameters(runtime_state.build_sdr_params(), force=force)

    def enqueue_parameters(
        self,
        sdr_parameters: Mapping[str, SdrParameterValue],
        *,
        force: bool = False,
    ) -> bool:
        """检查完整参数快照并加入发送队列。

        返回 True 表示快照发生变化且已经进入队列；False 表示与上次入队快照相同。
        """
        parameter_snapshot = self._normalize_parameter_update(sdr_parameters)

        with self._queue_lock:
            if "rx_gain" not in parameter_snapshot:
                if not force and self._last_enqueued_control_snapshot == parameter_snapshot:
                    return False

                self._put_snapshot(parameter_snapshot)
                self._last_enqueued_control_snapshot = dict(parameter_snapshot)
                return True

            if not force and self._last_enqueued_gain_snapshot == parameter_snapshot:
                return False

            self._put_snapshot(parameter_snapshot)
            self._last_enqueued_gain_snapshot = dict(parameter_snapshot)
            return True

    # ==================== 自适应 RX gain ====================

    def _handle_rx_power_features(self, features: Mapping[str, float]) -> None:
        """根据流图 telemetry 更新 RX gain，并把新快照加入发送队列。"""
        parsed_features = self._sanitize_rx_power_features(features)
        if parsed_features is None:
            return

        self._update_runtime_power_state(parsed_features)
        target_gain = self._calculate_next_rx_gain(parsed_features)
        if target_gain is None or target_gain == self._current_rx_gain:
            return

        self.enqueue_parameters({"rx_gain": target_gain})
        self._last_rx_gain_update_time = time.perf_counter()

    def _calculate_next_rx_gain(self, features: Mapping[str, float]) -> int | None:
        """根据三个通道平均功率中的最大项一步计算下一步 RX gain。"""
        now = time.perf_counter()
        if now - self._last_rx_gain_update_time < RX_GAIN_UPDATE_DEADZONE_SEC:
            return None

        max_avg_power_db = max(
            features["raw_peak_db"],
            features["interfere_avg_density_db"],
            features["info_avg_density_db"],
        )

        if RX_AVG_DEADZONE_LOW_DB <= max_avg_power_db <= RX_AVG_DEADZONE_HIGH_DB:
            return None

        gain_delta = int(round(RX_AVG_TARGET_DB - max_avg_power_db))
        if gain_delta == 0:
            gain_delta = 1 if max_avg_power_db < RX_AVG_TARGET_DB else -1

        return max(RX_GAIN_MIN, min(RX_GAIN_MAX, self._current_rx_gain + gain_delta))

    @staticmethod
    def _sanitize_rx_power_features(features: Mapping[str, float]) -> dict[str, float] | None:
        """校验 telemetry 中的三个功率特征。"""
        required_names = (
            "raw_peak_db",
            "interfere_avg_density_db",
            "info_avg_density_db",
        )
        parsed: dict[str, float] = {}
        for name in required_names:
            try:
                value = float(features[name])
            except (KeyError, TypeError, ValueError):
                return None
            if not math.isfinite(value):
                return None
            parsed[name] = value
        return parsed

    # ==================== 队列发送线程 ====================

    def _send_queue_worker(self) -> None:
        """按固定间隔从队列取出一条快照并发送到 GNU Radio。"""
        while not self._stop_event.is_set():
            try:
                snapshot = self._parameter_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            try:
                self._send_snapshot(snapshot)
            except Exception as exc:
                with self._queue_lock:
                    self._last_enqueued_control_snapshot = None
                    self._last_enqueued_gain_snapshot = None
                if self._runtime_state is not None:
                    self._runtime_state.log(f"SDR parameter send failed: {exc}")
                time.sleep(max(0.0, float(self.xmlrpc_config.queue_send_interval_sec)))
                continue

            self._last_sent_snapshot = dict(snapshot)
            if "rx_gain" in snapshot:
                self._current_rx_gain = int(snapshot["rx_gain"])
                if self._runtime_state is not None:
                    self._runtime_state.rx_gain.current_gain = self._current_rx_gain
            time.sleep(max(0.0, float(self.xmlrpc_config.queue_send_interval_sec)))

    def _send_snapshot(self, parameter_snapshot: Mapping[str, SdrParameterValue]) -> None:
        """把完整参数快照写入 GNU Radio XMLRPC setter。"""
        proxy = ServerProxy(
            f"http://{self.xmlrpc_config.host}:{self.xmlrpc_config.port}",
            allow_none=True,
        )

        for parameter_name, parameter_value in parameter_snapshot.items():
            setter = getattr(proxy, f"set_{parameter_name}")
            setter(parameter_value)

    def _put_snapshot(self, parameter_snapshot: SdrParameterSnapshot) -> None:
        """向队列写入快照；队列满时丢弃最旧快照。"""
        try:
            self._parameter_queue.put_nowait(dict(parameter_snapshot))
        except queue.Full:
            try:
                self._parameter_queue.get_nowait()
            except queue.Empty:
                pass
            self._parameter_queue.put_nowait(dict(parameter_snapshot))

    # ==================== telemetry 接收线程 ====================

    def _telemetry_worker(self) -> None:
        """接收 GNU Radio 输出的 RX 功率特征 float32 向量。"""
        try:
            import zmq
        except ImportError:
            return

        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.SUBSCRIBE, b"")
        socket.connect(
            f"tcp://{self.xmlrpc_config.telemetry_host}:{self.xmlrpc_config.telemetry_port}"
        )

        try:
            while not self._stop_event.is_set():
                try:
                    payload = socket.recv(flags=zmq.NOBLOCK)
                except zmq.Again:
                    time.sleep(0.05)
                    continue

                features = self._decode_rx_power_payload(payload)
                if features is not None:
                    self._handle_rx_power_features(features)
        finally:
            socket.close(0)
            context.term()

    # ==================== 参数快照工具 ====================

    def _normalize_parameter_update(
        self,
        sdr_parameters: Mapping[str, SdrParameterValue],
    ) -> SdrParameterSnapshot:
        """规范化 SDR 参数更新。

        普通控制参数保持快照语义：任意一个快照变量变化时，都必须把整组快照变量一同入队。
        rx_gain 是自适应闭环的独立增量项，只允许单独入队。
        """
        if not sdr_parameters:
            raise KeyError("SDR 参数更新不能为空")

        allowed_names = set(self.SDR_PARAMETER_NAMES) | {"rx_gain"}
        unknown_names = [name for name in sdr_parameters if name not in allowed_names]
        if unknown_names:
            joined = ", ".join(unknown_names)
            raise KeyError(f"SDR 参数更新包含未知字段: {joined}")

        parameter_names = set(sdr_parameters)
        if parameter_names == {"rx_gain"}:
            return {"rx_gain": sdr_parameters["rx_gain"]}

        if "rx_gain" in parameter_names:
            raise KeyError("rx_gain 必须作为独立增量入队，不能混入控制快照")

        missing_names = [name for name in self.SDR_PARAMETER_NAMES if name not in sdr_parameters]
        if missing_names:
            joined = ", ".join(missing_names)
            raise KeyError(f"SDR 控制快照缺少字段: {joined}")

        return {name: sdr_parameters[name] for name in self.SDR_PARAMETER_NAMES}

    @staticmethod
    def _decode_rx_power_payload(payload: bytes) -> dict[str, float] | None:
        """解码流图 ZMQ PUB 直接发布的 3 个 float32 功率值。"""
        if len(payload) < 12:
            return None

        raw_peak_db, interfere_avg_density_db, info_avg_density_db = struct.unpack(
            "<3f",
            payload[:12],
        )
        return {
            "raw_peak_db": float(raw_peak_db),
            "interfere_avg_density_db": float(interfere_avg_density_db),
            "info_avg_density_db": float(info_avg_density_db),
        }

    def _update_runtime_power_state(self, features: Mapping[str, float]) -> None:
        """把最新功率观测写入 RuntimeState，供网页显示。"""
        if self._runtime_state is None:
            return

        self._runtime_state.rx_power.raw_peak_db = features["raw_peak_db"]
        self._runtime_state.rx_power.interfere_avg_density_db = features[
            "interfere_avg_density_db"
        ]
        self._runtime_state.rx_power.info_avg_density_db = features["info_avg_density_db"]
        self._runtime_state.rx_power.updated_at = time.time()


__all__ = [
    "SdrParameterController",
    "SdrParameterSnapshot",
    "SdrParameterValue",
    "SdrXmlRpcConfig",
]
