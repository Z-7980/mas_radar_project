import numpy as np

class FortifyManager:
    """2027 规则 5.2：累计命中≥10 发后可消耗加固机会选圆心。
    命中流 (x,y,区域类别) 由雷达无线链路(SDR 侧)注入 feed_hit；
    半径 A 由服务器按 ρ 计算并经 0x020E 回传，本模块只决定“何时+圆心放哪”。
    坐标系：装甲模块检测面左下角原点，单位 mm——注意不是场地坐标！"""
    def __init__(self):
        self.hits = []          # [(x, y)] 己方弹丸命中对方装甲的坐标流
        self.fortify_cnt = 0    # 0x0121 byte8 计数器（按小局清零）
    def feed_hit(self, x, y, zone):        # SDR 接口回调点（协议落点待定，先走本地回环/UDP）
        self.hits.append((x, y))
    def sigma(self):                        # 散布圆半径：命中点到中位点距离的中位数
        if len(self.hits) < 10: return None
        pts = np.asarray(self.hits)
        med = np.median(pts, axis=0)
        return float(np.median(np.linalg.norm(pts - med, axis=1)))
    def decide(self, chance_left, eta_ally, eta_enemy):
        """返回 (x,y) 表示请求加固；返回 None 表示不动作。
        策略示例：机会≥1 且散布紧(σ 小)时，把圆心放对方集火方向的薄弱装甲板中心（区域中位数）"""
        if chance_left <= 0 or len(self.hits) < 10:
            return None
        rho = eta_ally / max(eta_enemy, 1e-6)          # ρ = Λ己/Λ敌 的近似（Λ=η/σ 完整版按需扩展）
        if rho < 1.2:                                   # 收益不足不烧机会
            return None
        c = np.median(np.asarray(self.hits), axis=0)    # 保守策略：以命中密集区为圆心
        self.fortify_cnt += 1
        return (int(c[0]), int(c[1]))
