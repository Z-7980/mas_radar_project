# -*- coding: utf-8 -*-
"""RMUC2026-NC 运行时状态管理

本模块负责整个软件节点的共享状态维护、运行计数、日志缓存，
以及 RF 参数的实时计算与导出。

设计边界：
- RuntimeState 是 TX、RX、GUI、SDR 控制面共享的唯一状态源；
- GUI 的 /api/state 直接读取 RuntimeState.to_dict()；
- sdr_control.py 只从 RuntimeState.build_sdr_params() 获取完整 SDR 同步参数；
- RX 通道只采用网页强制选择，不再在 state 层保留自动能量选择数据。
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import time

from .constants import (
    BIT_RATE_HZ,
    CMD_MAX_AIR_PACKETS,
    GFSK_BT,
    GFSK_GAUSSIAN_SPAN,
    GFSK_MODULATION,
    RF_PROFILES,
    RX_CENTER_FREQ_MAP,
    SAMPLE_RATE_HZ,
    SAMPLES_PER_SYMBOL,
    Team,
    TX_POWER_DBM_MAX,
    TX_POWER_DBM_MIN,
    WaveType,
)


# ==================== 类型定义 ====================

SdrParameterValue = int | float
SdrParameterSnapshot = dict[str, SdrParameterValue]


# ==================== 通用工具函数 ====================

def clamp_tx_power_dbm(power_dbm: int | float | str) -> int:
    """将 GUI 选择的发射功率限制在硬件调试允许范围内。

    NanoSDR/PlutoSDR 使用衰减值控制发射强度，当前工程约定：
        tx_attenuation = -tx_power_dbm

    参数:
        power_dbm: 输入功率值，可能来自 JSON，因此允许字符串或数值。

    返回:
        限制在 [TX_POWER_DBM_MIN, TX_POWER_DBM_MAX] 范围内的整数功率值。
    """
    return max(TX_POWER_DBM_MIN, min(TX_POWER_DBM_MAX, int(power_dbm)))


def _estimate_event_rate_hz(events: list[float], now: float, window_sec: float = 2.0) -> float:
    """计算过去 window_sec 秒内的平均事件频率。"""
    if not events:
        return 0.0

    cutoff = now - window_sec
    recent_events = [timestamp for timestamp in events if timestamp >= cutoff]
    if not recent_events:
        return 0.0
    if len(recent_events) == 1:
        elapsed_sec = now - recent_events[0]
        return 1.0 / elapsed_sec if elapsed_sec > 0 else 0.0

    span_sec = recent_events[-1] - recent_events[0]
    return (len(recent_events) - 1) / span_sec if span_sec > 0 else 0.0


# ==================== RF 参数计算函数 ====================

def current_rf_profile(
    team: Team,
    wave_type: WaveType,
    interference_level: int,
    tx_power_dbm: int | float | str | None = None,
) -> dict:
    """获取当前 TX RF 参数，并补充 MHz 单位的显示字段。

    参数:
        team: 发射阵营。
        wave_type: 当前发射波形类型。
        interference_level: 干扰等级，只有干扰波时有效。
        tx_power_dbm: 可选功率覆盖值；None 时使用规则默认值。

    返回:
        包含 freq_hz、bandwidth_hz、power_dbm、sensitivity、freq_mhz、bandwidth_mhz 的字典。
    """
    if wave_type == WaveType.INFO:
        profile = dict(RF_PROFILES[team][WaveType.INFO])
    else:
        level = max(1, min(3, int(interference_level)))
        profile = dict(RF_PROFILES[team][level])

    if tx_power_dbm is not None:
        profile["power_dbm"] = clamp_tx_power_dbm(tx_power_dbm)

    profile["freq_mhz"] = round(int(profile["freq_hz"]) / 1e6, 3)
    profile["bandwidth_mhz"] = round(int(profile["bandwidth_hz"]) / 1e6, 3)
    return profile


def current_modulation_profile() -> dict:
    """获取固定的 RMUC 空口调制参数，用于 GUI 状态展示。"""
    return {
        "modulation": GFSK_MODULATION,
        "sample_rate_hz": SAMPLE_RATE_HZ,
        "samples_per_symbol": SAMPLES_PER_SYMBOL,
        "bit_rate_hz": BIT_RATE_HZ,
        "bt": GFSK_BT,
        "gaussian_span": GFSK_GAUSSIAN_SPAN,
        "bit_order": "MSB-first",
    }


def parse_interference_key(interference_key: str | None) -> tuple[Team, int]:
    """解析干扰波选择 key。

    参数:
        interference_key: 形如 "red-1"、"blue-3" 的网页选择值。

    返回:
        (team, level) 元组；非法输入回落到红方 1 级。
    """
    try:
        team_text, level_text = str(interference_key or "red-1").split("-", 1)
        team = Team(team_text)
        level = int(level_text)
        if level not in (1, 2, 3):
            raise ValueError
        return team, level
    except (ValueError, TypeError):
        return Team.RED, 1


def make_interference_key(team: Team, level: int) -> str:
    """将干扰波阵营与等级格式化为网页使用的 key。"""
    return f"{team.value}-{max(1, min(3, int(level)))}"


def rx_center_freq_hz(team: Team, level: int) -> int:
    """根据 RX 当前选择的阵营和干扰等级获取硬件中心频点。

    1~2 级：使用干扰波自身频率中心；
    3 级：红方为红方信息波中心 - 0.1 MHz，蓝方为蓝方信息波中心 + 0.1 MHz。
    """
    return int(RX_CENTER_FREQ_MAP[team][max(1, min(3, int(level)))])


def rx_profiles_from_selection(rx_selection: "RxSelectionConfig | None") -> tuple[dict, dict]:
    """根据网页强制选择返回 RX 干扰波支路和信息波支路 RF 参数。

    信息波阵营跟随当前干扰波选择的阵营；是否真正启用信息波解调由
    rx_int_bw 和 GNU Radio 流图统一解调支路共同决定。
    """
    interference_key = rx_selection.interference_current if rx_selection else "red-1"
    team, level = parse_interference_key(interference_key)

    interference_profile = dict(RF_PROFILES[team][level])
    interference_profile["team"] = team
    interference_profile["level"] = level

    info_profile = dict(RF_PROFILES[team][WaveType.INFO])
    info_profile["team"] = team

    return interference_profile, info_profile


# ==================== 数据模型 ====================

@dataclass
class TxConfig:
    """网页发送区维护的当前发送配置。

    这些字段会被 TX Worker 读取，并生成 0x0A01 ~ 0x0A06 对应的业务 payload。
    """

    team: Team = Team.RED
    wave_type: WaveType = WaveType.INTERFERE
    interference_level: int = 1
    tx_power_dbm: int = -10
    # 第二发射源通过独立 ZMQ 端口和流图中的第二 NanoSDR sink 发送信息波。
    second_source_enabled: bool = False
    second_tx_power_dbm: int = -60

    randomize: bool = False
    enabled_cmds: dict[str, bool] = field(default_factory=lambda: {
        "0x0A06": True,
        "0x0A01": True,
        "0x0A02": True,
        "0x0A03": True,
        "0x0A04": True,
        "0x0A05": True,
    })

    secret_key: str = "ABC123"
    positions: list[tuple[int, int]] = field(default_factory=lambda: [(150, 150)] * 6)
    hps: list[int] = field(default_factory=lambda: [500, 500, 250, 250, 400])
    ammo: list[int] = field(default_factory=lambda: [100, 50, 50, 500, 200])

    remain_coins: int = 200
    total_coins: int = 600
    status_flags: int = 0

    buff_hero: list[int] = field(default_factory=lambda: [0, 0, 0, 0, 0])
    buff_eng: list[int] = field(default_factory=lambda: [0, 0, 0, 0, 0])
    buff_inf3: list[int] = field(default_factory=lambda: [0, 0, 0, 0, 0])
    buff_inf4: list[int] = field(default_factory=lambda: [0, 0, 0, 0, 0])
    buff_sentry: list[int] = field(default_factory=lambda: [0, 0, 0, 0, 0, 1])
    # 0x0A05 V2 新增的主要状态，顺序为英雄、工程、步兵3、步兵4、哨兵。
    robot_statuses: list[int] = field(default_factory=lambda: [0, 0, 0, 0, 0])


@dataclass
class RxSelectionConfig:
    """RX 通道选择状态。

    当前工程只采用网页强制选择：
    - interference_forced：网页下拉框选择值；
    - interference_current：实际应用的干扰波通道；
    - info_current_team：信息波阵营，固定跟随干扰波阵营。

    不再保留自动能量选择、滞回切换或 min_energy 字段。
    """

    interference_forced: str = "red-1"
    interference_current: str = "red-1"
    info_current_team: str = "red"

    def apply_forced_interference(self, interference_key: str | None) -> None:
        """应用网页强制选择，并同步当前干扰波通道与信息波阵营。"""
        team, level = parse_interference_key(interference_key)
        normalized_key = make_interference_key(team, level)
        self.interference_forced = normalized_key
        self.interference_current = normalized_key
        self.info_current_team = team.value


@dataclass
class RxGainConfig:
    """RX 硬件增益状态。

    current_gain 是写入 GNU Radio rx_gain 变量的手动硬件增益值。
    当前默认由 sdr_control.py 的自适应闭环更新，网页快照只负责提供完整控制上下文。
    """

    current_gain: int = 40


@dataclass
class RxPowerTelemetry:
    """RX 自适应增益使用的功率观测值。"""

    raw_peak_db: float | None = None
    interfere_avg_density_db: float | None = None
    info_avg_density_db: float | None = None
    updated_at: float | None = None


@dataclass
class RuntimeCounters:
    """调试节点运行计数器。"""

    serial_frames_tx: int = 0
    air_frames_tx: int = 0

    tx_bytes_sent: int = 0
    tx_bits_sent: int = 0
    rx_unpacked_bytes: int = 0

    air_payloads_rx: int = 0
    serial_frames_rx: int = 0
    serial_frames_failed: int = 0

    @property
    def parse_success_rate(self) -> float:
        """计算串口业务帧解析成功率。"""
        total = self.serial_frames_rx + self.serial_frames_failed
        if total == 0:
            return 1.0
        return self.serial_frames_rx / total


@dataclass
class RuntimeState:
    """整个软件节点的共享状态。

    GUI 的 /api/state 直接读取这里的数据；发送线程、接收线程和 HTTP 线程
    也都围绕该对象读写状态。
    """

    mode: str
    tx: TxConfig = field(default_factory=TxConfig)
    rx_selection: RxSelectionConfig = field(default_factory=RxSelectionConfig)
    rx_gain: RxGainConfig = field(default_factory=RxGainConfig)
    rx_power: RxPowerTelemetry = field(default_factory=RxPowerTelemetry)
    rx: dict = field(default_factory=dict)
    counters: RuntimeCounters = field(default_factory=RuntimeCounters)

    packets_by_cmd: dict = field(default_factory=dict)
    cmd_stats: dict = field(default_factory=dict)
    rx_diag_stats: dict = field(default_factory=dict)
    updated_seq: dict[str, int] = field(default_factory=dict)
    logs: list[str] = field(default_factory=list)

    _seq_counter: int = field(default=0, init=False, repr=False)
    """内部更新序号，用于前端字段闪烁高亮。"""

    # ==================== 日志与高亮 ====================

    def log(self, message: str) -> None:
        """追加一条运行日志，并限制日志列表长度。"""
        stamp = time.strftime("%H:%M:%S")
        self.logs.append(f"[{stamp}] {message}")
        self.logs = self.logs[-200:]

    def mark_updated(self, keys: list[str]) -> None:
        """标记被成功解码包触碰的字段，用于 GUI 高亮闪烁。"""
        for key in keys:
            self._seq_counter += 1
            self.updated_seq[key] = self._seq_counter

    # ==================== GUI 状态导出 ====================

    def to_dict(self) -> dict:
        """转换成可 JSON 序列化的状态字典。

        这里会顺手刷新每个包 ID 的接收频率，并把 Enum、RF 参数和 SDR 参数
        转换成网页可直接使用的格式。
        """
        self.refresh_cmd_rates()
        self.refresh_rx_diag_rates()

        data = asdict(self)
        data.pop("_seq_counter", None)

        data["tx"]["team"] = self.tx.team.value
        data["tx"]["wave_type"] = self.tx.wave_type.value
        data["tx"]["rf_profile"] = current_rf_profile(
            self.tx.team,
            self.tx.wave_type,
            self.tx.interference_level,
            self.tx.tx_power_dbm,
        )
        data["tx"]["modulation_profile"] = current_modulation_profile()

        rx_team, rx_level = parse_interference_key(self.rx_selection.interference_current)
        data["rx"]["current_frequency_hz"] = rx_center_freq_hz(rx_team, rx_level)
        data["rx"]["current_frequency_mhz"] = round(
            data["rx"]["current_frequency_hz"] / 1e6,
            3,
        )

        data["sdr_params"] = self.build_sdr_params()
        data["counters"]["parse_success_rate"] = self.counters.parse_success_rate
        data["cmd_max_air_packets"] = CMD_MAX_AIR_PACKETS
        return data

    # ==================== SDR 参数快照 ====================

    def build_sdr_params(self) -> SdrParameterSnapshot:
        """生成与 GNU Radio XMLRPC setter 完全匹配的 SDR 参数快照。

        本方法是 sdr_control.py 的唯一参数来源。字段名必须与 GNU Radio 流图变量名
        和 SdrParameterController.SDR_PARAMETER_NAMES 保持一致。
        """
        sdr_params: SdrParameterSnapshot = {}
        sdr_params.update(self._build_tx_sdr_params())
        sdr_params.update(self._build_rx_sdr_params())
        return sdr_params

    def _build_tx_sdr_params(self) -> SdrParameterSnapshot:
        """生成 TX 侧 SDR 参数。

        默认源的中心频率跟随当前波形；第二源只承载同阵营信息波，
        因此必须使用独立的信息波中心频率，不能跟随默认源的干扰等级。
        """
        primary_tx_profile = current_rf_profile(
            self.tx.team,
            self.tx.wave_type,
            self.tx.interference_level,
            self.tx.tx_power_dbm,
        )
        second_tx_profile = current_rf_profile(
            self.tx.team,
            WaveType.INFO,
            self.tx.interference_level,
            self.tx.second_tx_power_dbm,
        )

        is_info_wave = self.tx.wave_type == WaveType.INFO
        is_interfere_wave = self.tx.wave_type == WaveType.INTERFERE
        tx_level = max(1, min(3, int(self.tx.interference_level)))

        return {
            "tx_center_freq1": int(primary_tx_profile["freq_hz"]),
            "tx_center_freq2": int(second_tx_profile["freq_hz"]),
            "tx_attenuation1": int(-primary_tx_profile["power_dbm"]),
            "tx_attenuation2": int(-second_tx_profile["power_dbm"]),
            "tx_mod_info_en": 1.0 if is_info_wave else 0.0,
            "tx_mod_int1_en": 1.0 if (is_interfere_wave and tx_level == 1) else 0.0,
            "tx_mod_int2_en": 1.0 if (is_interfere_wave and tx_level == 2) else 0.0,
            "tx_mod_int3_en": 1.0 if (is_interfere_wave and tx_level == 3) else 0.0,
        }

    def _build_rx_sdr_params(self) -> SdrParameterSnapshot:
        """生成 RX 侧 SDR 参数。

        RX 参数完全由网页强制选择的 interference_current 决定，不依赖 TX 当前阵营或
        TX 当前波形类型。这样 TX 发红方、RX 解蓝方等调试场景也能得到正确频点。
        """
        interference_profile, info_profile = rx_profiles_from_selection(self.rx_selection)
        rx_team = interference_profile["team"]
        rx_level = int(interference_profile["level"])

        return {
            "rx_center_freq": rx_center_freq_hz(rx_team, rx_level),
            "rx_interfere_channel_freq": int(interference_profile["freq_hz"]),
            "rx_info_channel_freq": int(info_profile["freq_hz"]),
            "rx_int_bw": int(interference_profile["bandwidth_hz"]),
        }

    # ==================== 统计刷新 ====================

    def refresh_cmd_rates(self, window_sec: float = 60.0) -> None:
        """用滑动时间窗口统计每个包 ID 的实时接收频率。

        如果某个包在窗口内没有新事件，频率会回落到 0，避免旧频率一直停留。
        """
        now = time.perf_counter()
        for stats in self.cmd_stats.values():
            events = [ts for ts in stats.get("events", []) if now - ts <= window_sec]
            stats["events"] = events
            stats["rate_hz"] = _estimate_event_rate_hz(events, now, window_sec)

    def record_rx_diag_event(self, key: str) -> dict:
        """记录 RX 诊断事件，用于滑动窗口频率显示。"""
        now = time.perf_counter()
        stats = self.rx_diag_stats.setdefault(
            key,
            {"count": 0, "rate_hz": 0.0, "events": []},
        )
        stats["count"] += 1
        stats.setdefault("events", []).append(now)
        return stats

    def refresh_rx_diag_rates(self, window_sec: float = 60.0) -> None:
        """刷新 RX 诊断事件频率，使用与包频率相同的滑动窗口。"""
        now = time.perf_counter()
        for stats in self.rx_diag_stats.values():
            events = [ts for ts in stats.get("events", []) if now - ts <= window_sec]
            stats["events"] = events
            stats["rate_hz"] = _estimate_event_rate_hz(events, now, window_sec)


__all__ = [
    "RuntimeCounters",
    "RuntimeState",
    "RxGainConfig",
    "RxSelectionConfig",
    "SdrParameterSnapshot",
    "SdrParameterValue",
    "TxConfig",
    "clamp_tx_power_dbm",
    "current_modulation_profile",
    "current_rf_profile",
    "make_interference_key",
    "parse_interference_key",
    "rx_center_freq_hz",
    "rx_profiles_from_selection",
]
