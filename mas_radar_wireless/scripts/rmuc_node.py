# -*- coding: utf-8 -*-
"""RMUC2026-NC 协议软件节点入口文件

本文件是整个软件节点的启动入口，负责解析命令行参数、分别构建 SDR 数据面配置
与 SDR 控制面配置，创建协议节点实例并启动 Web GUI 服务器。

重构边界：
- ZMQ 数据面配置由 sdr_link.NanoSdrGnuRadioConfig 表示；
- XMLRPC 控制面配置由 sdr_control.SdrXmlRpcConfig 表示；
- 入口文件只负责组装配置，不直接操作 ZMQ、XMLRPC 或 SDR 参数计算。
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from typing import Callable

# ==================== 路径配置 ====================
# 支持从工程根目录外直接执行本文件，例如：python rmuc_node.py。
PROJECT_ROOT = Path(__file__).resolve().parents[1]
GRC_DIR = PROJECT_ROOT / "grc"
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from rmuc.constants import (  # noqa: E402
    DEFAULT_TX_RATE_HZ,
    GUI_DEFAULT_PORT,
    LOCAL_HOST,
    RX_INFO_DEFAULT_PORT,
    RX_INTERFERE_DEFAULT_PORT,
    RX_POWER_TELEMETRY_DEFAULT_PORT,
    TX_DEFAULT_PORT,
    TX_SECOND_DEFAULT_PORT,
    XMLRPC_DEFAULT_PORT,
)
from rmuc.flowgraph_runtime import resolve_flowgraph_python  # noqa: E402
from rmuc.node import SoftwareProtocolNode  # noqa: E402
from rmuc.sdr_control import SdrXmlRpcConfig  # noqa: E402
from rmuc.sdr_link import NanoSdrGnuRadioConfig  # noqa: E402
from rmuc.web_gui import GuiServer  # noqa: E402
from rmuc.udp_bridge import RadarUdpBridge  # noqa: E402


# ==================== 命令行参数 ====================

def build_argument_parser() -> argparse.ArgumentParser:
    """构建命令行参数解析器。

    本入口只暴露运行模式、发送速率、Web 端口、ZMQ 端口和 XMLRPC 端口。
    主机地址统一使用 constants.LOCAL_HOST，避免同一工程内出现多套地址配置。
    """
    parser = argparse.ArgumentParser(description="RMUC2026-NC protocol software node")

    # 运行模式：debug 使用纯软件虚拟链路；其余模式使用 GNU Radio/NanoSDR 数据链路。
    parser.add_argument(
        "--mode",
        choices=("debug", "loopback", "tx", "rx"),
        default="debug",
        help="运行模式：debug/loopback/tx/rx",
    )

    # 软件 TX 周期生成速率。规则推送周期当前为 10 Hz。
    parser.add_argument(
        "--rate-hz",
        dest="tx_rate_hz",
        type=float,
        default=DEFAULT_TX_RATE_HZ,
        help="软件 TX 生成周期频率，单位 Hz",
    )

    # Web GUI 监听端口。
    parser.add_argument(
        "--port",
        dest="gui_port",
        type=int,
        default=GUI_DEFAULT_PORT,
        help="Web GUI 监听端口",
    )

    # TX ZMQ 端口：Python 作为 PUB，GNU Radio 作为 SUB。
    parser.add_argument(
        "--sdr-tx-port",
        dest="tx_zmq_port",
        type=int,
        default=TX_DEFAULT_PORT,
        help="Python -> GNU Radio 的 TX ZMQ PUB 端口",
    )
    parser.add_argument(
        "--sdr-tx-second-port",
        dest="tx_second_zmq_port",
        type=int,
        default=TX_SECOND_DEFAULT_PORT,
        help="Python -> GNU Radio 的第二 TX ZMQ PUB 端口",
    )

    # RX 信息波 ZMQ 端口：GNU Radio 作为 PUB，Python 作为 SUB。
    parser.add_argument(
        "--sdr-rx-info-port",
        dest="rx_info_zmq_port",
        type=int,
        default=RX_INFO_DEFAULT_PORT,
        help="GNU Radio -> Python 的信息波 RX ZMQ SUB 端口",
    )

    # RX 干扰波 ZMQ 端口：GNU Radio 作为 PUB，Python 作为 SUB。
    parser.add_argument(
        "--sdr-rx-interfere-port",
        dest="rx_interfere_zmq_port",
        type=int,
        default=RX_INTERFERE_DEFAULT_PORT,
        help="GNU Radio -> Python 的干扰波 RX ZMQ SUB 端口",
    )

    # XMLRPC 端口：Python 作为客户端，GNU Radio xmlrpc_server 作为服务端。
    parser.add_argument(
        "--sdr-xmlrpc-port",
        dest="sdr_xmlrpc_port",
        type=int,
        default=XMLRPC_DEFAULT_PORT,
        help="Python -> GNU Radio 的 XMLRPC 参数同步端口",
    )

    # GNU Radio 流图守护：非 debug 模式默认随 rmuc_node 一起启动并自动重启。
    parser.add_argument(
        "--flowgraph-script",
        dest="flowgraph_script",
        default="auto",
        help="需要守护的 GNU Radio 生成脚本路径；auto 表示自动查找主力流图脚本",
    )
    parser.add_argument(
        "--no-flowgraph-watchdog",
        dest="flowgraph_watchdog",
        action="store_false",
        default=True,
        help="禁用 GNU Radio 流图进程守护",
    )
    parser.add_argument(
        "--flowgraph-python",
        dest="flowgraph_python",
        default=None,
        help="指定运行 GNU Radio 流图的 Python；未指定时使用 ~/radioconda/bin/python3",
    )
    # ===== A机↔B机 UDP 桥（rmuc/udp_bridge.py）=====
    parser.add_argument("--udp-peer-host", dest="udp_peer_host", default="192.168.3.10",
                        help="A机(视觉/决策)业务网IP；B机上行数据发往此处")
    parser.add_argument("--udp-peer-port", dest="udp_peer_port", type=int, default=53200,
                        help="A机接收B机上行的UDP端口")
    parser.add_argument("--udp-listen-port", dest="udp_listen_port", type=int, default=53201,
                        help="本机接收A机解调配置的UDP端口")
    parser.add_argument("--no-udp-bridge", dest="udp_bridge", action="store_false", default=True,
                        help="禁用A↔B UDP桥（TX实例/单机分段验证时使用）")
    
    return parser


# ==================== 配置构造 ====================

def build_sdr_link_config(args: argparse.Namespace) -> NanoSdrGnuRadioConfig:
    """构造 SDR 数据面链路配置。

    该配置仅描述 ZMQ 数据流端口，不包含 XMLRPC 控制字段。
    debug 模式下 node 可以选择忽略该配置并改用 VirtualSdrLink。
    """
    return NanoSdrGnuRadioConfig(
        tx_zmq_host=LOCAL_HOST,
        tx_zmq_port=args.tx_zmq_port,
        tx_second_zmq_port=args.tx_second_zmq_port,
        rx_zmq_host=LOCAL_HOST,
        rx_info_zmq_port=args.rx_info_zmq_port,
        rx_interfere_zmq_port=args.rx_interfere_zmq_port,
    )


def build_sdr_control_config(args: argparse.Namespace) -> SdrXmlRpcConfig:
    """构造 SDR 控制面 XMLRPC 配置。

    该配置仅用于 sdr_control.SdrParameterController，不参与 ZMQ 数据收发。
    """
    return SdrXmlRpcConfig(
        host=LOCAL_HOST,
        port=args.sdr_xmlrpc_port,
        telemetry_host=LOCAL_HOST,
        telemetry_port=RX_POWER_TELEMETRY_DEFAULT_PORT,
    )


# ==================== GNU Radio 流图守护 ====================

class FlowgraphWatchdog:
    """GNU Radio 生成脚本守护线程。

    本类只负责启动、观察和重启流图进程；SDR 参数同步仍由 SoftwareProtocolNode
    内部的 SdrParameterController 通过 XMLRPC 完成。
    """

    def __init__(
        self,
        *,
        script_path: Path,
        python_executable: Path,
        on_process_started: Callable[[], None] | None = None,
    ):
        """保存守护线程配置。

        参数:
            script_path: GNU Radio Companion 生成的 Python 流图脚本路径。
        """
        self.script_path = script_path.resolve()
        self.python_executable = python_executable.resolve()
        self.restart_interval_sec = 2.0
        self.on_process_started = on_process_started

        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._process: subprocess.Popen | None = None

    def start(self) -> None:
        """启动守护线程。"""
        if self._thread is not None and self._thread.is_alive():
            return

        self._thread = threading.Thread(
            target=self._run,
            name="gnu-radio-flowgraph-watchdog",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        """停止守护线程，并终止当前由守护线程启动的流图进程。"""
        self._stop_event.set()
        self._terminate_process()

        if self._thread is not None:
            self._thread.join(timeout=3.0)

    def _run(self) -> None:
        """守护主循环：流图不存在时启动，退出后按间隔自动重启。"""
        while not self._stop_event.is_set():
            self._process = self._start_process()

            while not self._stop_event.is_set():
                if self._process.poll() is not None:
                    break
                time.sleep(0.5)

            if self._stop_event.is_set():
                break

            time.sleep(self.restart_interval_sec)

        self._terminate_process()

    def _start_process(self) -> subprocess.Popen:
        """启动一次 GNU Radio 流图子进程。"""
        process = subprocess.Popen(
            [self.python_executable, str(self.script_path)],
            cwd=str(self.script_path.parent),
            env=self._build_subprocess_env(),
        )
        if self.on_process_started is not None:
            threading.Timer(1.0, self.on_process_started).start()
        return process

    def _build_subprocess_env(self) -> dict[str, str]:
        """构造流图子进程环境，确保 grc 目录中的 block_*.py 可以被导入。"""
        env = dict(os.environ)

        pythonpath_parts = [
            str(self.script_path.parent),
            str(PROJECT_ROOT),
        ]
        existing_pythonpath = env.get("PYTHONPATH")
        if existing_pythonpath:
            pythonpath_parts.append(existing_pythonpath)

        env["PYTHONPATH"] = os.pathsep.join(pythonpath_parts)
        env.setdefault("PYTHONIOENCODING", "utf-8")
        return env

    def _terminate_process(self) -> None:
        """终止当前流图子进程。"""
        process = self._process
        if process is None or process.poll() is not None:
            return

        process.terminate()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3.0)


def resolve_flowgraph_script(script_arg: str) -> Path | None:
    """解析 GNU Radio 生成脚本路径。

    auto 模式会优先查找当前主力 SIC 流图生成脚本；若 GRC 使用内部 id 生成文件名，
    则回落到 RMUC2026_NanoSDR_TRX.py。
    """
    if str(script_arg).strip().lower() != "auto":
        script_path = Path(script_arg)
        if not script_path.is_absolute():
            script_path = PROJECT_ROOT / script_path
        return script_path

    candidates = (
        GRC_DIR / "RMUC2026_NanoSDR_TRX_SIC.py",
        GRC_DIR / "RMUC2026_NanoSDR_TRX.py",
    )
    for candidate in candidates:
        if candidate.exists():
            return candidate

    return None


def maybe_start_flowgraph_watchdog(
    args: argparse.Namespace,
    node: SoftwareProtocolNode,
) -> FlowgraphWatchdog | None:
    """按运行模式和命令行参数决定是否启动 GNU Radio 流图守护。"""
    if args.mode == "debug" or not args.flowgraph_watchdog:
        return None

    script_path = resolve_flowgraph_script(args.flowgraph_script)
    if script_path is None:
        node.state.log("GNU Radio flowgraph watchdog skipped: generated script was not found.")
        return None

    if not script_path.exists():
        node.state.log(f"GNU Radio flowgraph watchdog skipped: script not found: {script_path}")
        return None

    python_probe = resolve_flowgraph_python(args.flowgraph_python)
    if not python_probe.ok:
        node.state.log(f"GNU Radio flowgraph watchdog skipped: {python_probe.reason}")
        return None

    watchdog = FlowgraphWatchdog(
        script_path=script_path,
        python_executable=python_probe.path,
        on_process_started=lambda: node.sync_sdr_params(force=True),
    )
    watchdog.start()
    node.state.log(
        f"GNU Radio flowgraph watchdog started: {script_path}, "
        f"python={python_probe.path}, gnuradio={python_probe.version}"
    )
    return watchdog


# ==================== 主入口 ====================

def main() -> None:
    """主入口函数。

    执行流程：
    1. 解析命令行参数；
    2. 分别构建 SDR 数据面配置和控制面配置；
    3. 创建并启动协议节点；
    4. 启动 Web GUI 服务器；
    5. 主线程保持运行，等待键盘中断。
    """
    args = build_argument_parser().parse_args()

    sdr_link_config = build_sdr_link_config(args)
    sdr_control_config = build_sdr_control_config(args)

    # 说明：SoftwareProtocolNode 需要在后续重构中接收这两类配置，
    # 不再从 NanoSdrGnuRadioConfig 中读取 XMLRPC 字段。
    node = SoftwareProtocolNode(
        mode=args.mode,
        rate_hz=args.tx_rate_hz,
        sdr_link_config=sdr_link_config,
        sdr_control_config=sdr_control_config,
    )
    flowgraph_watchdog = maybe_start_flowgraph_watchdog(args, node)
    node.start()


    # A↔B 有线UDP桥：上行 packet/hit/status，下行 config（失效自动空操作）
    udp_bridge = None
    if args.udp_bridge:
        udp_bridge = RadarUdpBridge(
            node,
            peer_host=args.udp_peer_host,
            peer_port=args.udp_peer_port,
            listen_port=args.udp_listen_port,
        )
        udp_bridge.start()

    # 启动模式日志，便于从 GUI 日志区确认当前链路形态。
    if args.mode == "tx":
        node.state.log("TX mode: only the transmitter worker is active.")
    elif args.mode == "rx":
        node.state.log("RX mode: only the receiver worker is active.")
    elif args.mode == "loopback":
        node.state.log("Loopback mode: TX and RX workers are active through the SDR/GNU Radio path.")
    else:
        node.state.log("Debug mode: TX and RX workers are connected by the in-memory SDR queue.")

    gui_server = GuiServer(node, host=LOCAL_HOST, port=args.gui_port)
    gui_thread = threading.Thread(
        target=gui_server.serve_forever,
        name="rmuc-gui",
        daemon=True,
    )
    gui_thread.start()

    try:
        while gui_thread.is_alive():
            time.sleep(0.5)
    except KeyboardInterrupt:
        node.stop()
        if flowgraph_watchdog is not None:
            flowgraph_watchdog.stop()
        if gui_server.httpd is not None:
            gui_server.httpd.shutdown()
    finally:
        if udp_bridge is not None:
            udp_bridge.stop()
        if flowgraph_watchdog is not None:
            flowgraph_watchdog.stop()



if __name__ == "__main__":
    main()
