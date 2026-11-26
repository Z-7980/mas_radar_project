# -*- coding: utf-8 -*-
"""RMUC2026-NC SDR 数据链路层

本模块负责 Python 与 GNU Radio 之间的 ZMQ 数据流传输。
控制面（XMLRPC 参数同步）由 sdr_control.py 负责，本模块只处理数据面。

链路模型：
- TX：Python 作为 ZMQ PUB，GNU Radio 作为 ZMQ SUB 读取 packed byte 字节；
- RX：GNU Radio 作为 ZMQ PUB，Python 作为 ZMQ SUB 读取解调后的 unpacked byte 字节流。

设计边界：
- 本模块不直接访问 NanoSDR 硬件，NanoSDR 由 GNU Radio 流图通过 IP 访问；
- 本模块不下发频点、功率、门控等控制参数；
- 本模块不把 RX unpacked bytes 转换成 list[int]，bit 语义解释交给 RX 解码层完成；
- debug 模式使用 VirtualSdrLink，真实 SDR 模式使用 NanoSdrGnuRadioLink。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import queue
import time
from typing import Any

from .constants import (
    LOCAL_HOST,
    RX_INFO_DEFAULT_PORT,
    RX_INTERFERE_DEFAULT_PORT,
    TX_DEFAULT_PORT,
    TX_SECOND_DEFAULT_PORT,
    WaveType,
)


# ==================== 数据格式 ====================

class TxChannel(str, Enum):
    """TX 物理发射源通道。"""

    PRIMARY = "primary"
    SECONDARY = "secondary"


@dataclass(frozen=True)
class SdrUnpackedByteChunk:
    """RX 与 SDR 链路之间的数据块。

    unpacked_bytes 保留 GNU Radio 解调链路的原始输出格式：
    - 每个 byte 表示 1 个已判决 bit；
    - byte 取值通常为 0x00 或 0x01；
    - 本层不再转换为 list[int]，也不做 bit 语义再解释。

    后续 RX 解码层如需 bit list，可在 RxWorker 或 AirFrameDecoder 入口处
    使用 [byte & 0x01 for byte in unpacked_bytes] 进行转换。
    """

    wave_type: WaveType | None
    unpacked_bytes: bytes


# ==================== ZMQ 数据面配置 ====================

@dataclass(frozen=True)
class NanoSdrGnuRadioConfig:
    """NanoSDR/GNU Radio ZMQ 数据链路配置。

    本配置只描述 Python 与 GNU Radio 之间的 ZMQ 数据面连接，
    不包含 XMLRPC 控制面字段。

    连接关系：
    - TX：Python 绑定 tx_zmq_host:tx_zmq_port，GNU Radio 连接该 PUB 端口；
    - RX：Python 连接 rx_zmq_host:rx_info_zmq_port 与 rx_interfere_zmq_port，
      GNU Radio 分别在这两个端口发布信息波和干扰波的 unpacked bytes。
    """

    tx_zmq_host: str = LOCAL_HOST
    tx_zmq_port: int = TX_DEFAULT_PORT
    tx_second_zmq_port: int = TX_SECOND_DEFAULT_PORT
    rx_zmq_host: str = LOCAL_HOST
    rx_info_zmq_port: int = RX_INFO_DEFAULT_PORT
    rx_interfere_zmq_port: int = RX_INTERFERE_DEFAULT_PORT


# ==================== 链路抽象 ====================

class SdrLink:
    """SDR 数据链路抽象基类。

    TX 侧统一发送 packed bytes；
    RX 侧统一返回 GNU Radio 风格的 unpacked bytes。

    注意：
    - 本层只负责搬运数据，不负责协议解码；
    - RX 返回的 unpacked bytes 中，每个 byte 表示 1 个判决 bit；
    - 是否转成 list[int] 应由 RX 解码层自行决定。
    """

    def send_bytes(
        self,
        wave_type: WaveType | None,
        frame_bytes: bytes,
        tx_channel: TxChannel = TxChannel.PRIMARY,
    ) -> None:
        """发送一段空口帧字节流。

        TX 侧直接传输 packed bytes，由 GNU Radio 流图内部完成 MSB-first 展开。
        tx_channel 对应真实 ZMQ 发射端口；wave_type 用于虚拟链路保留包来源。
        """
        _ = tx_channel
        raise NotImplementedError

    def recv_unpacked_bytes(self) -> SdrUnpackedByteChunk | None:
        """接收一段解调后的 unpacked bytes；没有数据时返回 None。"""
        raise NotImplementedError

    def close(self) -> None:
        """关闭链路资源。"""
        return


# ==================== 虚拟链路 ====================

class VirtualSdrLink(SdrLink):
    """纯软件虚拟 SDR 数据链路。

    TX 线程写入完整空口帧 packed bytes；
    RX 线程从队列读取后，将 packed bytes 转换为 GNU Radio 风格的 unpacked bytes。

    debug 模式下用于验证软件协议流程，不模拟丢包、噪声、频偏、延迟等物理层特性。
    虚拟链路保留 wave_type 字段，因此上层仍能区分信息波与干扰波。
    """

    _QUEUE_MAXSIZE = 10000
    """队列容量上限，防止 TX 持续写入而 RX 不消费时无限增长。"""

    def __init__(self):
        self._packed_byte_queues: dict[WaveType | None, queue.Queue[bytes]] = {
            WaveType.INTERFERE: queue.Queue(maxsize=self._QUEUE_MAXSIZE),
            WaveType.INFO: queue.Queue(maxsize=self._QUEUE_MAXSIZE),
            None: queue.Queue(maxsize=self._QUEUE_MAXSIZE),
        }
        self._rx_poll_order: tuple[WaveType | None, ...] = (
            WaveType.INTERFERE,
            WaveType.INFO,
            None,
        )
        self._rx_poll_start_index = 0

    # ==================== 对外接口 ====================

    def send_bytes(
        self,
        wave_type: WaveType | None,
        frame_bytes: bytes,
        tx_channel: TxChannel = TxChannel.PRIMARY,
    ) -> None:
        """将 packed bytes 写入虚拟队列；队列满时丢弃最新数据。"""
        _ = tx_channel
        if not frame_bytes:
            return

        target_queue = self._packed_byte_queues.get(wave_type, self._packed_byte_queues[None])
        try:
            target_queue.put_nowait(frame_bytes)
        except queue.Full:
            # debug 链路只用于调试，队列满说明 RX 消费不足；此处直接丢弃以避免内存膨胀。
            pass

    def recv_unpacked_bytes(self) -> SdrUnpackedByteChunk | None:
        """从虚拟队列读取 packed bytes，并转换为 unpacked bytes 后返回。

        转换规则与 GNU Radio digital_gfsk_mod 的 MSB-first 展开语义保持一致：
        每个原始 byte 按 bit7 -> bit0 顺序展开，每个 bit 存为一个 byte。
        """
        queue_count = len(self._rx_poll_order)
        start_index = self._rx_poll_start_index
        self._rx_poll_start_index = (self._rx_poll_start_index + 1) % queue_count

        wave_type = None
        packed_bytes = None
        for offset in range(queue_count):
            queue_index = (start_index + offset) % queue_count
            candidate_wave_type = self._rx_poll_order[queue_index]
            candidate_queue = self._packed_byte_queues[candidate_wave_type]
            try:
                packed_bytes = candidate_queue.get_nowait()
            except queue.Empty:
                continue
            wave_type = candidate_wave_type
            break

        if packed_bytes is None:
            return None

        return SdrUnpackedByteChunk(
            wave_type=wave_type,
            unpacked_bytes=self._bytes_to_unpacked_bytes_msb(packed_bytes),
        )

    # ==================== 内部工具 ====================

    @staticmethod
    def _bytes_to_unpacked_bytes_msb(packed_bytes: bytes) -> bytes:
        """将 packed bytes 按 bit7 -> bit0 顺序展开为 unpacked bytes。"""
        return bytes(
            (byte >> bit_index) & 0x01
            for byte in packed_bytes
            for bit_index in range(7, -1, -1)
        )


# ==================== GNU Radio ZMQ 链路 ====================

class NanoSdrGnuRadioLink(SdrLink):
    """Python 与 GNU Radio 流图之间的 ZMQ TCP/IP 数据链路。

    预期连接关系：
    - TX：Python 作为 ZMQ PUB，GNU Radio 作为 ZMQ SUB 读取 packed byte 字节；
    - RX：GNU Radio 作为 ZMQ PUB，Python 作为 ZMQ SUB 读取 unpacked byte 字节流。

    RX 默认输出格式：
    - GNU Radio 解调链路输出 unpacked bytes；
    - 每个 byte 通常为 0x00 或 0x01；
    - 本层直接原样返回 payload，不转换为 list[int]。
    """

    _RX_POLL_IDLE_SLEEP_SEC = 0.05
    """RX 无数据时的短暂休眠时间，避免接收线程空转占满 CPU。"""

    _PUB_SUB_CONNECT_DELAY_SEC = 0.1
    """PUB socket 绑定后的短暂等待时间，降低 SUB 尚未连接导致首包丢失的概率。"""

    def __init__(self, link_config: NanoSdrGnuRadioConfig):
        self.link_config = link_config

        self._zmq_module: Any | None = None
        self._zmq_context: Any | None = None

        self._tx_publisher_socket: Any | None = None
        self._tx_second_publisher_socket: Any | None = None
        self._rx_info_subscriber_socket: Any | None = None
        self._rx_interfere_subscriber_socket: Any | None = None

        self._rx_poll_start_index = 0

    # ==================== 对外接口 ====================

    def send_bytes(
        self,
        wave_type: WaveType | None,
        frame_bytes: bytes,
        tx_channel: TxChannel = TxChannel.PRIMARY,
    ) -> None:
        """通过 ZMQ PUB 发送 packed bytes 给 GNU Radio。

        tx_channel 对应主/副两个物理发射源端口；wave_type 只描述该端口当前承载的波形。
        """
        if not frame_bytes:
            return

        tx_socket = self._ensure_tx_publisher_socket(tx_channel)
        tx_socket.send(frame_bytes)

    def recv_unpacked_bytes(self) -> SdrUnpackedByteChunk | None:
        """轮询两个 RX ZMQ SUB socket，返回最先到达的 unpacked bytes。

        采用轮换优先策略：每次调用时交替起始端口，避免单一端口持续优先导致
        另一个端口数据积压。

        注意：
        - GNU Radio 侧已经输出 unpacked byte 字节流；
        - 本层不再执行 [byte & 0x01 for byte in payload]；
        - 如果后续协议解码层需要 list[int]，应在 RX Worker 中转换。
        """
        rx_sockets = self._ensure_rx_subscriber_sockets()

        socket_count = len(rx_sockets)
        start_index = self._rx_poll_start_index
        self._rx_poll_start_index = (self._rx_poll_start_index + 1) % socket_count

        for offset in range(socket_count):
            socket_index = (start_index + offset) % socket_count
            wave_type, rx_socket = rx_sockets[socket_index]

            try:
                payload = rx_socket.recv(flags=self._zmq_module.NOBLOCK)
            except self._zmq_module.Again:
                continue

            return SdrUnpackedByteChunk(
                wave_type=wave_type,
                unpacked_bytes=bytes(payload),
            )

        time.sleep(self._RX_POLL_IDLE_SLEEP_SEC)
        return None

    def close(self) -> None:
        """关闭所有 ZMQ socket，并终止本链路持有的 ZMQ context。"""
        for zmq_socket in (
            self._tx_publisher_socket,
            self._tx_second_publisher_socket,
            self._rx_info_subscriber_socket,
            self._rx_interfere_subscriber_socket,
        ):
            if zmq_socket is not None:
                zmq_socket.close(0)

        self._tx_publisher_socket = None
        self._tx_second_publisher_socket = None
        self._rx_info_subscriber_socket = None
        self._rx_interfere_subscriber_socket = None

        if self._zmq_context is not None:
            self._zmq_context.term()

        self._zmq_context = None
        self._zmq_module = None

    # ==================== ZMQ 懒加载 ====================

    def _ensure_zmq_context(self):
        """懒加载 pyzmq 并创建本链路专用的 ZMQ context。"""
        if self._zmq_context is None:
            try:
                import zmq
            except ImportError as exc:
                raise RuntimeError(
                    "NanoSdrGnuRadioLink 需要 pyzmq；请安装 pyzmq 或使用 debug 模式"
                ) from exc

            self._zmq_module = zmq
            self._zmq_context = zmq.Context()

        return self._zmq_context

    def _ensure_tx_publisher_socket(self, tx_channel: TxChannel):
        """懒创建 TX ZMQ PUB socket，并根据物理发射源选择端口。"""
        if tx_channel == TxChannel.SECONDARY:
            if self._tx_second_publisher_socket is None:
                self._tx_second_publisher_socket = self._bind_tx_publisher_socket(
                    self.link_config.tx_second_zmq_port,
                )
            return self._tx_second_publisher_socket

        if self._tx_publisher_socket is None:
            self._tx_publisher_socket = self._bind_tx_publisher_socket(
                self.link_config.tx_zmq_port,
            )
        return self._tx_publisher_socket

    def _bind_tx_publisher_socket(self, zmq_port: int):
        """绑定单个 TX ZMQ PUB socket。"""
        context = self._ensure_zmq_context()

        tx_socket = context.socket(self._zmq_module.PUB)
        tx_socket.setsockopt(self._zmq_module.LINGER, 0)
        tx_socket.bind(f"tcp://{self.link_config.tx_zmq_host}:{zmq_port}")

        # PUB/SUB 建链不是瞬时完成，绑定后短暂等待可降低首包丢失概率。
        time.sleep(self._PUB_SUB_CONNECT_DELAY_SEC)
        return tx_socket

    def _ensure_rx_subscriber_sockets(self) -> list[tuple[WaveType, Any]]:
        """懒创建两个 RX ZMQ SUB socket，分别连接信息波和干扰波输出端口。"""
        if self._rx_info_subscriber_socket is None:
            self._rx_info_subscriber_socket = self._connect_rx_subscriber_socket(
                self.link_config.rx_info_zmq_port,
            )

        if self._rx_interfere_subscriber_socket is None:
            self._rx_interfere_subscriber_socket = self._connect_rx_subscriber_socket(
                self.link_config.rx_interfere_zmq_port,
            )

        return [
            (WaveType.INFO, self._rx_info_subscriber_socket),
            (WaveType.INTERFERE, self._rx_interfere_subscriber_socket),
        ]

    def _connect_rx_subscriber_socket(self, zmq_port: int):
        """创建单个 RX ZMQ SUB socket，并连接到 GNU Radio PUB 端口。"""
        context = self._ensure_zmq_context()

        rx_socket = context.socket(self._zmq_module.SUB)
        rx_socket.setsockopt(self._zmq_module.LINGER, 0)
        rx_socket.setsockopt(self._zmq_module.SUBSCRIBE, b"")
        rx_socket.connect(f"tcp://{self.link_config.rx_zmq_host}:{zmq_port}")

        return rx_socket


__all__ = [
    "NanoSdrGnuRadioConfig",
    "NanoSdrGnuRadioLink",
    "SdrLink",
    "SdrUnpackedByteChunk",
    "TxChannel",
    "VirtualSdrLink",
]
