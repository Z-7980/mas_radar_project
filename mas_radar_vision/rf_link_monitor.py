# -*- coding: utf-8 -*-
"""A 机侧独立无线电链路监控壳（新文件）。

不启动完整 main.py（无需 torch/MVS/相机），只跑 rf_link 客户端 + UI 面板，
用于快速验证 A↔B UDP 链路和全部接收数据。依赖仅：cv2、numpy、标准库。

用法:
    python rf_link_monitor.py                      # 双机联调，默认发往 192.168.3.20
    python rf_link_monitor.py --peer 127.0.0.1     # 单机同机 UDP（配合本机 B 端程序/探针）
    python rf_link_monitor.py --own R --level 2    # 选边与目标干扰等级

按键:
    1 / 2 / 3        切换要求 B 机解调的干扰等级（set_target_level）
    r / b            切换己方阵营（对方阵营自动取反）
    q 或 ESC         退出
"""

import argparse

import cv2

from rf_link import RfLinkClient
from rf_ui_panel import draw_rf_panel


def main():
    ap = argparse.ArgumentParser(description="A机侧 rf_link 独立监控壳")
    ap.add_argument("--peer", default="192.168.3.20", help="B 机 IP（同机联调填 127.0.0.1）")
    ap.add_argument("--peer-port", type=int, default=53201)
    ap.add_argument("--listen-port", type=int, default=53200)
    ap.add_argument("--own", default="B", choices=("R", "B", "r", "b"), help="本机阵营")
    ap.add_argument("--level", type=int, default=1, choices=(1, 2, 3), help="目标干扰等级")
    ap.add_argument("--interval", type=float, default=1.0, help="config 下发周期(s)")
    args = ap.parse_args()

    client = RfLinkClient(
        own_team=args.own.upper(),
        peer_host=args.peer,
        peer_port=args.peer_port,
        listen_host="0.0.0.0",
        listen_port=args.listen_port,
        config_interval=args.interval,
    )
    client.set_target_level(args.level)
    client.start()
    print("[monitor] rf_link 启动：listen 0.0.0.0:%d -> peer %s:%d, own=%s, opp=%s, level=%d"
          % (args.listen_port, args.peer, args.peer_port, client.own_team,
             client.opp_team, client.target_level))
    if not client.ok:
        print("[monitor] 注意：监听端口初始化失败，面板将显示 DISABLED（端口 53200 被 main.py 占用？）")

    cv2.namedWindow("rf_link_monitor", cv2.WINDOW_NORMAL)
    try:
        while True:
            cv2.imshow("rf_link_monitor", draw_rf_panel(client))
            key = cv2.waitKey(50) & 0xFF
            if key in (ord("q"), 27):
                break
            elif key in (ord("1"), ord("2"), ord("3")):
                client.set_target_level(key - ord("0"))
                print("[monitor] target_level -> %d" % client.target_level)
            elif key in (ord("r"), ord("b")):
                client.own_team = chr(key).upper()
                client.opp_team = "red" if client.own_team == "B" else "blue"
                print("[monitor] own -> %s, opp -> %s" % (client.own_team, client.opp_team))
    finally:
        client.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
