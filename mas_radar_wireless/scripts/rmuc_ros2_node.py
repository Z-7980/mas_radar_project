#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RMUC2026-NC ROS2 启动入口。

本入口面向外部 ROS2 launch / ros2 run 调用：
- 暴露 mode、no-flowgraph-watchdog 和 flowgraph-python 三个启动参数；
- 其余 ZMQ、XMLRPC、Web GUI 端口全部使用当前工程默认值；
- 与 scripts/rmuc_node.py 共用 rmuc 包内核心逻辑，避免 ROS2 与非 ROS 调试路径分叉。
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

import rclpy

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


# ==================== 路径配置 ====================

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
from rmuc.ros2_bridge import RadarParseEmWaveRos2Bridge  # noqa: E402
from rmuc.sdr_control import SdrXmlRpcConfig  # noqa: E402
from rmuc.sdr_link import NanoSdrGnuRadioConfig  # noqa: E402
from rmuc.web_gui import GuiServer  # noqa: E402


# ==================== 命令行参数 ====================

def build_argument_parser() -> argparse.ArgumentParser:
    """构建 ROS2 启动入口参数。"""
    parser = argparse.ArgumentParser(description="RMUC2026-NC ROS2 protocol node")
    parser.add_argument(
        "--mode",
        choices=("debug", "loopback", "tx", "rx"),
        default="debug",
        help="运行模式：debug/loopback/tx/rx",
    )
    parser.add_argument(
        "--no-flowgraph-watchdog",
        dest="flowgraph_watchdog",
        action="store_false",
        default=True,
        help="禁用 GNU Radio 流图进程守护，适用于外部单独启动流图的场景",
    )
    parser.add_argument(
        "--flowgraph-python",
        dest="flowgraph_python",
        default=None,
        help="指定运行 GNU Radio 流图的 Python；未指定时使用 ~/radioconda/bin/python3",
    )
    return parser


# ==================== 默认配置构造 ====================

def build_sdr_link_config() -> NanoSdrGnuRadioConfig:
    """构造默认 SDR 数据面链路配置。"""
    return NanoSdrGnuRadioConfig(
        tx_zmq_host=LOCAL_HOST,
        tx_zmq_port=TX_DEFAULT_PORT,
        tx_second_zmq_port=TX_SECOND_DEFAULT_PORT,
        rx_zmq_host=LOCAL_HOST,
        rx_info_zmq_port=RX_INFO_DEFAULT_PORT,
        rx_interfere_zmq_port=RX_INTERFERE_DEFAULT_PORT,
    )


def build_sdr_control_config() -> SdrXmlRpcConfig:
    """构造默认 SDR XMLRPC 控制面配置。"""
    return SdrXmlRpcConfig(
        host=LOCAL_HOST,
        port=XMLRPC_DEFAULT_PORT,
        telemetry_host=LOCAL_HOST,
        telemetry_port=RX_POWER_TELEMETRY_DEFAULT_PORT,
    )


# ==================== GNU Radio 流图守护 ====================

class FlowgraphWatchdog:
    """GNU Radio 生成脚本守护线程。"""

    def __init__(
        self,
        *,
        script_path: Path,
        python_executable: Path,
        on_process_started: Callable[[], None] | None = None,
    ):
        """保存流图脚本路径和进程启动后的回调。"""
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
        """停止守护线程并终止当前流图进程。"""
        self._stop_event.set()
        self._terminate_process()
        if self._thread is not None:
            self._thread.join(timeout=3.0)

    def _run(self) -> None:
        """流图退出后按固定间隔自动重启。"""
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
        """启动一次 GNU Radio 生成脚本。"""
        process = subprocess.Popen(
            [self.python_executable, str(self.script_path)],
            cwd=str(self.script_path.parent),
            env=self._build_subprocess_env(),
        )
        if self.on_process_started is not None:
            threading.Timer(1.0, self.on_process_started).start()
        return process

    def _build_subprocess_env(self) -> dict[str, str]:
        """构造流图子进程环境，保证自定义 block 和 rmuc 包可导入。"""
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
        """终止当前由守护线程启动的流图进程。"""
        process = self._process
        if process is None or process.poll() is not None:
            return
        process.terminate()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3.0)


def resolve_flowgraph_script() -> Path | None:
    """查找主力 GNU Radio 生成脚本。

    源码运行时优先查找工程 grc 目录；安装后运行时额外查找包 share 目录。
    """
    search_dirs = [GRC_DIR]
    if get_package_share_directory is not None:
        try:
            search_dirs.append(Path(get_package_share_directory("radar_parse_em_wave")) / "grc")
        except Exception:
            pass

    for grc_dir in search_dirs:
        for candidate_name in (
            "RMUC2026_NanoSDR_TRX_SIC.py",
            "RMUC2026_NanoSDR_TRX.py",
        ):
            candidate = grc_dir / candidate_name
            if candidate.exists():
                return candidate
    return None


def maybe_start_flowgraph_watchdog(
    *,
    mode: str,
    enabled: bool,
    node: SoftwareProtocolNode,
    flowgraph_python: str | None = None,
) -> FlowgraphWatchdog | None:
    """按 ROS2 入口参数决定是否启动 GNU Radio 流图守护。"""
    if mode == "debug" or not enabled:
        return None

    script_path = resolve_flowgraph_script()
    if script_path is None:
        node.state.log("GNU Radio flowgraph watchdog skipped: generated script was not found.")
        return None

    python_probe = resolve_flowgraph_python(flowgraph_python)
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
    """启动协议核心、Web GUI、ROS2 bridge，并进入 ROS2 spin。"""
    args, ros_args = build_argument_parser().parse_known_args()

    protocol_node = SoftwareProtocolNode(
        mode=args.mode,
        rate_hz=DEFAULT_TX_RATE_HZ,
        sdr_link_config=build_sdr_link_config(),
        sdr_control_config=build_sdr_control_config(),
    )
    flowgraph_watchdog = maybe_start_flowgraph_watchdog(
        mode=args.mode,
        enabled=args.flowgraph_watchdog,
        node=protocol_node,
        flowgraph_python=args.flowgraph_python,
    )
    protocol_node.start()

    gui_server = GuiServer(protocol_node, host=LOCAL_HOST, port=GUI_DEFAULT_PORT)
    gui_thread = threading.Thread(
        target=gui_server.serve_forever,
        name="rmuc-gui",
        daemon=True,
    )
    gui_thread.start()

    rclpy.init(args=ros_args)
    ros2_bridge = RadarParseEmWaveRos2Bridge(protocol_node)

    try:
        while rclpy.ok():
            rclpy.spin_once(ros2_bridge.ros_node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        ros2_bridge.destroy()
        if rclpy.ok():
            rclpy.shutdown()

        protocol_node.stop()
        if flowgraph_watchdog is not None:
            flowgraph_watchdog.stop()
        if gui_server.httpd is not None:
            gui_server.httpd.shutdown()


if __name__ == "__main__":
    main()
