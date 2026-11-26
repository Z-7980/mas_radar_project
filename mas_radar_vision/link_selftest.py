# -*- coding: utf-8 -*-
"""A 机（mas_radar_temp）链路自检探针——不依赖主程序，用于验证 A↔B 物理链路与协议互通。

用法（默认即 A 机角色）:
    python link_selftest.py                     # 持续运行，Ctrl+C 退出
    python link_selftest.py --peer 192.168.3.20 --duration 10

行为:
    监听 0.0.0.0:53200 收 B 机上行的 packet/hit/status；
    每 1s 向 B 机 53201 发一条 config{"team":"red","interference_level":1}。
    收到的每条消息打印类型/序号/cmd/等级；退出时打印收发统计与丢包判定。

与主程序关系: 端口与正式链路相同(53200/53201)。自检时请先关闭 main.py 与 rmuc_node.py，
避免端口占用；自检通过后再启动正式程序。若端口被占用本程序会明确提示并退出。
"""

import argparse
import json
import socket
import sys
import time


def main():
    ap = argparse.ArgumentParser(description="A机↔B机 UDP 链路自检 (A 端)")
    ap.add_argument("--peer", default="192.168.3.20", help="B 机 IP")
    ap.add_argument("--peer-port", type=int, default=53201)
    ap.add_argument("--listen-port", type=int, default=53200)
    ap.add_argument("--duration", type=float, default=0, help="秒，0=一直运行")
    args = ap.parse_args()

    try:
        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx.bind(("0.0.0.0", args.listen_port))
        rx.settimeout(0.5)
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    except OSError as exc:
        print("[A自测] 端口占用或创建失败（请先关闭 main.py）：%s" % exc)
        sys.exit(2)

    peer = (args.peer, args.peer_port)
    seq = 0
    recv_by_type = {}
    last_status_ts = None
    t_end = time.time() + args.duration if args.duration > 0 else None
    print("[A自测] 监听 0.0.0.0:%d，向 %s:%d 发 config(red-1)，Ctrl+C 退出"
          % (args.listen_port, args.peer, args.peer_port))

    try:
        while True:
            # 1) 收
            try:
                raw, addr = rx.recvfrom(65535)
                msg = json.loads(raw.decode("utf-8"))
                mtype = msg.get("type", "?")
                recv_by_type[mtype] = recv_by_type.get(mtype, 0) + 1
                detail = ""
                if mtype == "packet":
                    detail = " cmd=%s" % msg.get("cmd")
                elif mtype == "status":
                    detail = " team=%s level=%s" % (msg.get("rx_team"), msg.get("rx_level"))
                    last_status_ts = time.time()
                elif mtype == "hit":
                    detail = " x=%s y=%s region=%s" % (msg.get("x"), msg.get("y"), msg.get("region"))
                print("[A自测] <== 来自 %s type=%s seq=%s%s" % (addr[0], mtype, msg.get("seq"), detail))
            except socket.timeout:
                pass
            except (ValueError, UnicodeDecodeError):
                print("[A自测] <!! 坏包（非 JSON），已忽略")

            # 2) 发（每 1s 一条 config）
            now = time.time()
            if int(now) != getattr(main, "_last_send", -1):
                seq += 1
                payload = {"v": 1, "type": "config", "seq": seq, "ts": now,
                           "team": "red", "interference_level": 1}
                try:
                    tx.sendto(json.dumps(payload).encode("utf-8"), peer)
                    main._last_send = int(now)
                except OSError as exc:
                    print("[A自测] 发送失败（对端不可达？）：%s" % exc)

            if t_end is not None and time.time() >= t_end:
                break
    except KeyboardInterrupt:
        pass
    finally:
        rx.close()
        tx.close()

    total = sum(recv_by_type.values())
    print("\n[A自测] 统计：收到共 %d 条 %s" % (total, recv_by_type))
    if recv_by_type.get("status"):
        gap = time.time() - last_status_ts if last_status_ts else 0
        print("[A自测] 结论：链路 OK（最近 status 距今 %.1fs，B 端 1Hz 心跳）" % gap)
    else:
        print("[A自测] 结论：未收到 B 端任何消息 —— 请检查 B 端自检是否在跑、网线、IP/网段、防火墙")


if __name__ == "__main__":
    main()
