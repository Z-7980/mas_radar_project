# -*- coding: utf-8 -*-
"""A 机无线电情报 UI 面板（新文件，供 main.py 或独立监控壳调用）。

作用：把 RfLinkClient 收到的**全部**跨机数据绘制成 1080×300 的 OpenCV 图：
  - 头部：链路状态(LINK OK/LOST/DISABLED)、对端地址、三类消息计数、心跳时距、
    己方/对方阵营、目标干扰等级 vs B 机实际上报等级、破解密钥；
  - 主体：0x0A01~0x0A06 六格，每格显示最近一次解码数据 + 新鲜度着色
    （绿≤1.5s / 黄≤4s / 红>4s 或未收到）；
  - 底部：最近受击事件（x,y,zone@时距）。

main.py 接入方式（用户改动，见交付清单）：
    from rf_ui_panel import draw_rf_panel
    ...
    information_ui_show = np.vstack((information_ui_show, draw_rf_panel(rf_client)))
或独立窗口：
    cv2.imshow('rf_link', draw_rf_panel(rf_client))

本文件只读客户端属性、不产生任何副作用；即使 rf_client 初始化失败(ok=False)
也只画一帧灰屏，不会抛异常。
"""

import time

import cv2
import numpy as np

W = 1080
H = 300

_CELLS = (
    ("0x0A01", "对方位置"),
    ("0x0A02", "对方血量"),
    ("0x0A03", "对方弹量"),
    ("0x0A04", "场地经济"),
    ("0x0A05", "增益状态"),
    ("0x0A06", "干扰密钥"),
)
_META_KEYS = {"cmd", "payload_len", "length_valid"}

_GREEN = (0, 200, 0)
_YELLOW = (0, 200, 230)
_RED = (0, 0, 255)
_GRAY = (120, 120, 120)
_CYAN = (255, 220, 0)
_WHITE = (255, 255, 255)

_F = cv2.FONT_HERSHEY_SIMPLEX


def _age_color(age):
    if age is None:
        return _RED
    if age <= 1.5:
        return _GREEN
    if age <= 4.0:
        return _YELLOW
    return _RED


def _age_text(ts):
    if ts is None:
        return "never", None
    age = time.time() - ts
    if age < 1.0:
        return "%dms" % int(age * 1000), age
    return "%.1fs" % age, age


def _flatten(obj, max_items=6):
    """把 decoded 业务字典摊平成紧凑字符串项。"""
    items = []
    if not isinstance(obj, dict):
        return [str(obj)[:40]] if obj else []
    for key, val in obj.items():
        if key in _META_KEYS:
            continue
        if isinstance(val, dict):
            inner = ",".join(str(v) for v in val.values())
            items.append("%s(%s)" % (key, inner))
        elif isinstance(val, (list, tuple)):
            items.append("%s%s" % (key, list(val)))
        elif val is not None:
            items.append("%s=%s" % (key, val))
        if len(items) >= max_items:
            break
    return items


def _wrap_lines(items, width_chars=40, max_lines=4):
    lines = []
    cur = ""
    for it in items:
        trial = (cur + " " + it).strip()
        if len(trial) <= width_chars:
            cur = trial
        else:
            if cur:
                lines.append(cur)
            cur = it[:width_chars]
    if cur:
        lines.append(cur)
    return lines[:max_lines] or ["-"]


def draw_rf_panel(client, blank=False):
    """绘制面板。client 为 RfLinkClient 实例；blank=True 画占位灰图。"""
    img = np.zeros((H, W, 3), dtype=np.uint8)
    img[:] = (24, 24, 24)

    if blank or client is None:
        cv2.putText(img, "RF LINK  (rf_client disabled / no camera)", (10, 30), _F, 0.6, _GRAY, 1)
        return img

    if not getattr(client, "ok", False):
        cv2.putText(img, "RF LINK  DISABLED (socket init failed, main radar unaffected)",
                    (10, 30), _F, 0.6, _RED, 1)
        return img

    now = time.time()

    # ===== 头部 =====
    alive = bool(getattr(client, "link_alive", False))
    link_color = _GREEN if alive else _RED
    link_text = "LINK OK" if alive else "LINK LOST"
    cv2.rectangle(img, (8, 8), (150, 34), link_color, -1)
    cv2.putText(img, link_text, (14, 28), _F, 0.6, (0, 0, 0), 2)

    peer = "%s:%d" % client.peer_addr
    cv2.putText(img, "peer " + peer, (164, 28), _F, 0.5, _WHITE, 1)
    cv2.putText(img, "own %s -> demod opp %s" % (client.own_team, client.opp_team),
                (380, 28), _F, 0.5, _WHITE, 1)

    lvl = getattr(client, "opp_jam_level", None)
    tgt = getattr(client, "target_level", 1)
    lvl_text = "lvl %s(tgt %d)" % (str(lvl), tgt)
    cv2.putText(img, lvl_text, (600, 28), _F, 0.5, _CYAN, 1)

    key = getattr(client, "opp_secret_key", None)
    key_text = "PW " + (str(key) if key else "----")
    cv2.putText(img, key_text, (760, 28), _F, 0.5, _GREEN if key else _GRAY, 1)

    s_ts = getattr(client, "_last_status_ts", 0.0)
    age_s = None if s_ts <= 0 else now - s_ts
    txt, _ = _age_text(None if s_ts <= 0 else s_ts)
    cv2.putText(img, "hb " + txt, (940, 28), _F, 0.5, _age_color(age_s), 1)

    counts = "pkt %d  hit %d  st %d" % (
        getattr(client, "rx_packet_count", 0),
        getattr(client, "rx_hit_count", 0),
        getattr(client, "rx_status_count", 0),
    )
    cv2.putText(img, counts, (10, 54), _F, 0.45, _GRAY, 1)

    # ===== 主体 3列×2行 =====
    cell_w, cell_h = 356, 112
    top = 64
    rf_info = getattr(client, "rf_info", {})
    updated = getattr(client, "rf_updated_at", {})
    for index, (cmd, name) in enumerate(_CELLS):
        row, col = divmod(index, 3)
        x0 = 4 + col * (cell_w + 4)
        y0 = top + row * (cell_h + 4)
        cv2.rectangle(img, (x0, y0), (x0 + cell_w, y0 + cell_h), (60, 60, 60), 1)

        data = rf_info.get(cmd)
        ts = updated.get(cmd)
        age = None if ts is None else now - ts
        age_txt, _ = _age_text(ts)
        cv2.putText(img, "%s %s" % (cmd, name), (x0 + 6, y0 + 20), _F, 0.5, _CYAN, 1)
        cv2.putText(img, age_txt, (x0 + cell_w - 70, y0 + 20), _F, 0.45, _age_color(age), 1)

        if data:
            for li, line in enumerate(_wrap_lines(_flatten(data))):
                cv2.putText(img, line, (x0 + 6, y0 + 44 + li * 18), _F, 0.42, _WHITE, 1)
        else:
            cv2.putText(img, "(waiting)", (x0 + 6, y0 + 44), _F, 0.42, _GRAY, 1)

    # ===== 底部：最近命中 =====
    hits = list(getattr(client, "recent_hits", []))[-4:]
    if hits:
        parts = []
        for x, y, zone, ts in hits:
            parts.append("(%d,%d,z%d,%.1fs)" % (x, y, zone, now - ts))
        cv2.putText(img, "hits: " + " ".join(parts), (8, H - 10), _F, 0.42, _YELLOW, 1)
    else:
        cv2.putText(img, "hits: none (need 10 to unlock fortify)", (8, H - 10), _F, 0.42, _GRAY, 1)

    return img
