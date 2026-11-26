# -*- coding: utf-8 -*-
"""RMUC2026-NC 网页调试台 HTTP 服务器

前端页面外置到 rmuc/web/index.html，本文件只负责 REST API 路由和静态文件服务。

设计边界：
- web_gui.py 只处理 HTTP 请求与响应；
- 业务状态由 node.state 维护；
- TX/RX 配置更新由 SoftwareProtocolNode 提供统一接口；
- SDR 参数计算和同步不在本文件中完成。
"""

from __future__ import annotations

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from .constants import GUI_DEFAULT_PORT, LOCAL_HOST
from .node import SoftwareProtocolNode


# 静态文件目录：与 web_gui.py 同级的 web/ 目录。
_WEB_DIR = Path(__file__).resolve().parent / "web"
_HTML_PATH = _WEB_DIR / "index.html"


class GuiServer:
    """基于 Python 标准库 http.server 的网页调试台。"""

    def __init__(
        self,
        node: SoftwareProtocolNode,
        host: str = LOCAL_HOST,
        port: int = GUI_DEFAULT_PORT,
    ):
        """绑定一个软件协议节点，后续 HTTP API 都读写该节点状态。"""
        self.node = node
        self.host = host
        self.port = port
        self.httpd: ThreadingHTTPServer | None = None

    # ==================== 服务生命周期 ====================

    def serve_forever(self) -> None:
        """启动 HTTP 服务，提供网页、状态查询和配置更新接口。"""
        handler_class = self._make_handler_class()
        self.httpd = ThreadingHTTPServer((self.host, self.port), handler_class)
        print(f"RMUC GUI: http://{self.host}:{self.port}")
        self.httpd.serve_forever()

    def _make_handler_class(self) -> type[BaseHTTPRequestHandler]:
        """构造请求处理器类。

        Handler 需要访问当前 GuiServer 绑定的 SoftwareProtocolNode，因此通过闭包捕获
        node。拆成独立方法后，serve_forever() 只负责服务器生命周期，路由逻辑更清晰。
        """
        node = self.node

        class Handler(BaseHTTPRequestHandler):
            """请求处理器：路由分发、参数解析、响应构造。"""

            # ---------- 基础工具 ----------

            def log_message(self, fmt: str, *args) -> None:
                """关闭标准 HTTP 访问日志，避免控制台被轮询刷屏。"""
                return

            def _send(self, status: int, body: bytes, content_type: str) -> bool:
                """统一发送 HTTP 响应；客户端已断开时静默忽略。"""
                try:
                    self.send_response(status)
                    self.send_header("Content-Type", content_type)
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return True
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    return False

            def _send_json(self, status: int, data: dict) -> bool:
                """发送 JSON 响应；客户端已断开时静默忽略。"""
                body = json.dumps(data, ensure_ascii=False).encode("utf-8")
                return self._send(status, body, "application/json; charset=utf-8")

            def _read_json_body(self, max_length: int = 65536) -> dict:
                """读取并解析请求体为 JSON，带长度限制防恶意请求。"""
                length_text = self.headers.get("Content-Length", "0")
                try:
                    content_length = int(length_text)
                except ValueError:
                    raise ValueError(f"Invalid Content-Length: {length_text}")

                if content_length > max_length:
                    raise ValueError(f"Request body too large: {content_length} > {max_length}")
                if content_length == 0:
                    return {}

                payload = self.rfile.read(content_length)
                return json.loads(payload.decode("utf-8"))

            # ---------- GET 路由 ----------

            def do_GET(self) -> None:
                """处理 GET 请求：根路径返回前端页面，/api/state 返回状态 JSON。"""
                if self.path == "/":
                    self._handle_index()
                elif self.path == "/api/state":
                    self._handle_state()
                else:
                    self._send(404, b"not found", "text/plain; charset=utf-8")

            def _handle_index(self) -> None:
                """返回前端调试台页面。"""
                if not _HTML_PATH.exists():
                    self._send(500, b"index.html not found", "text/plain; charset=utf-8")
                    return
                self._send(200, _HTML_PATH.read_bytes(), "text/html; charset=utf-8")

            def _handle_state(self) -> None:
                """返回当前运行状态。"""
                self._send_json(200, node.state.to_dict())

            # ---------- POST 路由 ----------

            def do_POST(self) -> None:
                """处理 POST 请求：配置更新接口。"""
                handlers = {
                    "/api/tx-data": self._handle_tx_data,
                    "/api/sdr": self._handle_sdr_control,
                    "/api/rx": self._handle_rx_selection,
                }
                handler = handlers.get(self.path)
                if handler is None:
                    self._send(404, b"not found", "text/plain; charset=utf-8")
                    return

                try:
                    patch = self._read_json_body()
                    handler(patch)
                    self._send_json(200, {"ok": True})
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    return
                except json.JSONDecodeError as exc:
                    self._send_json(400, {"ok": False, "error": f"JSON parse error: {exc}"})
                except ValueError as exc:
                    self._send_json(400, {"ok": False, "error": str(exc)})
                except Exception as exc:
                    self._send_json(500, {"ok": False, "error": f"Internal error: {exc}"})

            # ---------- POST 业务处理 ----------

            def _handle_tx_data(self, patch: dict) -> None:
                """更新软件层 payload 数据，不触发 SDR 硬件同步。"""
                node.update_tx(patch)

            def _handle_sdr_control(self, patch: dict) -> None:
                """更新 SDR 控制快照，包括 TX 源选择和 RX 解调选择。"""
                node.update_sdr_control(patch)

            def _handle_rx_selection(self, patch: dict) -> None:
                """单独更新 RX 解调选择。"""
                node.update_rx_selection(patch)

        return Handler


__all__ = [
    "GuiServer",
]
