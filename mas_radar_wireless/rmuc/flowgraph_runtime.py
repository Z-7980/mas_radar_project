# -*- coding: utf-8 -*-
"""GNU Radio 流图运行时 Python 探测。

ROS2 节点通常运行在系统 Python 中，而 GNU Radio 流图可能安装在 conda 环境中。
本模块负责在启动流图子进程前选择 Python，并验证其能否导入流图依赖的 GNU Radio 模块。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import subprocess


DEFAULT_FLOWGRAPH_PYTHON = Path.home() / "radioconda" / "bin" / "python3"
"""未显式配置时使用的 Radioconda Python 默认路径。"""


@dataclass(frozen=True)
class FlowgraphPythonProbe:
    """单个 Python 候选项的探测结果。"""

    path: Path
    ok: bool
    version: str | None = None
    reason: str = ""


def resolve_flowgraph_python(explicit_python: str | None = None) -> FlowgraphPythonProbe:
    """按优先级选择唯一的 GNU Radio Python，并验证其运行能力。

    优先级：
    1. 命令行显式指定；
    2. 固定默认路径 ~/radioconda/bin/python3。

    已选路径验证失败时直接返回错误，不再搜索其他 Conda 环境、系统 Python
    或 PATH 中的 python3，避免不同电脑意外选中依赖不完整的解释器。
    """
    if explicit_python:
        candidate = Path(explicit_python).expanduser()
        source = "命令行 --flowgraph-python"
    else:
        candidate = DEFAULT_FLOWGRAPH_PYTHON
        source = "默认 Radioconda 路径"

    probe = probe_flowgraph_python(candidate)
    if probe.ok:
        return probe

    return FlowgraphPythonProbe(
        path=probe.path,
        ok=False,
        reason=f"{source} {probe.path} 验证失败: {probe.reason}",
    )


def probe_flowgraph_python(candidate: str | Path) -> FlowgraphPythonProbe:
    """检查一个 Python 是否具备运行当前 GNU Radio 流图的能力。"""
    path = Path(candidate).expanduser()
    if not path.exists():
        return FlowgraphPythonProbe(path=path, ok=False, reason="路径不存在")
    if not path.is_file():
        return FlowgraphPythonProbe(path=path, ok=False, reason="不是文件")

    # 实际导入流图当前依赖的 GNU Radio 组件；版本号只用于日志，不作为能力判断条件。
    script = (
        "from gnuradio import gr, iio, zeromq, qtgui\n"
        "version_fn = getattr(gr, 'version', None)\n"
        "print(version_fn() if callable(version_fn) else 'unknown')\n"
    )
    try:
        result = subprocess.run(
            [str(path), "-c", script],
            check=False,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
    except Exception as exc:
        return FlowgraphPythonProbe(path=path, ok=False, reason=f"探测失败: {exc}")

    if result.returncode != 0:
        detail = (result.stderr or result.stdout or "").strip().splitlines()
        reason = detail[-1] if detail else f"退出码 {result.returncode}"
        return FlowgraphPythonProbe(path=path, ok=False, reason=reason)

    version = (result.stdout or "").strip().splitlines()[-1]
    return FlowgraphPythonProbe(path=path, ok=True, version=version)


__all__ = [
    "DEFAULT_FLOWGRAPH_PYTHON",
    "FlowgraphPythonProbe",
    "probe_flowgraph_python",
    "resolve_flowgraph_python",
]

