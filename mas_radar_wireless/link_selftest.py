# -*- coding: utf-8 -*-
"""B 机（sp_rf_2026）链路自检探针——不依赖 GNU Radio/rmuc 主程序，用于验证 A↔B 物理链路与协议互通。

用法（默认即 B 机角色）:
    python3 link_selftest.py                    # 持续运行，Ctrl+C 退出
    python3 link_selftest.py --peer 192.168.3.10 --duration 10

行为:
    监听 0.0.0.0:53201 收 A 机下行的 config；
    每 1s 向 A 机 53200 发一组自测消息：status(心跳) + packet(0x0A06 样例密钥) + hit(样例命中)，
    用于同时验证 A 端三类上行消息的接收与显示。

与主程序关系: 端口与正式链路相同(53201)。自检时请先关闭 rmuc_node.py，避免端口占用；
自检通过后再启动正式程序。若端口被占用本脚本会明确提示并退出。
"""

import argparse
import json
import socket
import sys
import time

TEAM_LEVELS = [1, 2, 3]


def main():
    ap = argparse.ArgumentParser(description="A机↔B机 UDP 链路自检 (B 端)")
    ap.add_argument("--peer", default="192.168.3.10", help="A 机 IP")
    ap.add_argument("--peer-port", type=int, default=53200)
    ap.add_argument("--listen-port", type=int, default=53201)
    ap.add_argument("--duration", type=float, default=0, help="秒，0=一直运行")
    args = ap.parse_args()

    try:
        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx.bind(("0.0.0.0", args.listen_port))
        rx.settimeout(0.5)
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    except OSError as exc:
        print("[B自测] 端口占用或创建失败（请先关闭 rmuc_node.py）：%s" % exc)
        sys.exit(2)

    peer = (args.peer, args.peer_port)
    seq = 0
    tick = 0
    recv_by_type = {}
    last_config = None
    t_end = time.time() + args.duration if args.duration > 0 else None
    print("[B自测] 监听 0.0.0.0:%d，向 %s:%d 发 status/packet/hit，Ctrl+C 退出"
          % (args.listen_port, args.peer, args.peer_port))

    def send(msg):
        nonlocal seq
        seq += 1
        msg.update({"v": 1, "seq": seq, "ts": time.time()})
        try:
            tx.sendto(json.dumps(msg).encode("utf-8"), peer)
        except OSError as exc:
            print("[B自测] 发送失败（对端不可达？）：%s" % exc)

    try:
        while True:
            # 1) 收 config
            try:
                raw, addr = rx.recvfrom(65535)
                msg = json.loads(raw.decode("utf-8"))
                mtype = msg.get("type", "?")
                recv_by_type[mtype] = recv_by_type.get(mtype, 0) + 1
                if mtype == "config":
                    last_config = msg
                    print("[B自测] <== 来自 %s config team=%s level=%s"
                          % (addr[0], msg.get("team"), msg.get("interference_level")))
                else:
                    print("[B自测] <== 来自 %s type=%s seq=%s" % (addr[0], mtype, msg.get("seq")))
            except socket.timeout:
                pass
            except (ValueError, UnicodeDecodeError):
                print("[B自测] <!! 坏包（非 JSON），已忽略")

            # 2) 每 1s 发一组自测消息（status 常发；每 4s 追加一组 packet+hit 样例）
            now = time.time()
            if int(now) != getattr(main, "_last_send", -1):
                main._last_send = int(now)
                tick += 1
                level = TEAM_LEVELS[tick % 3] + 0  # 1..3 轮换，模拟等级跟踪
                send({"type": "status", "rx_team": "red", "rx_level": level,
                      "mode": "selftest", "parse_success_rate": 1.0, "alive": True})
                if tick % 4 == 0:
                    send({"type": "packet", "cmd": "0x0A06",
                          "data": {"cmd": "0x0A06", "key": "ABC123", "length_valid": True,
                                   "payload_len": 6}})
                    send({"type": "hit", "x": 300, "y": 450, "robot": "B3", "region": 2})

            if t_end is not None and time.time() >= t_end:
                break
    except KeyboardInterrupt:
        pass
    finally:
        rx.close()
        tx.close()

    print("\n[B自测] 统计：收到共 %d 条 %s" % (sum(recv_by_type.values()), recv_by_type))
    if recv_by_type.get("config"):
        print("[B自测] 结论：链路 OK（最近 config：%s level=%s）"
              % (last_config.get("team"), last_config.get("interference_level")))
    else:
        print("[B自测] 结论：未收到 A 端 config —— 请检查 A 端自测是否在跑、网线、IP/网段、防火墙")


if __name__ == "__main__":
    main()
