"""침로 제어 성능 지표 (HANDOFF.md 7장).

⚠ band 와 ss_frac 은 "무엇을 정상상태로 보고 무엇을 정정으로 볼 것인가"라는
  검증 조건이다. HANDOFF.md 5장에 따라 작업자가 결정할 항목이며, 여기 기본값은
  논의를 위한 출발점일 뿐이다. 확정 시 docs/decisions.md D-008 에 기록한다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from .sim import SimResult, ssa

__all__ = ["HeadingMetrics", "heading_metrics", "metrics_table"]

R2D = 180.0 / math.pi


@dataclass(frozen=True)
class HeadingMetrics:
    """모든 각도 단위는 deg.

    Attributes
    ----------
    rms_err_ss : 정상상태 침로 오차 RMS [deg]
    rms_err_all : 전 구간 침로 오차 RMS [deg]
    t_enter : ±band 대역 최초 진입 시각 [s]. HANDOFF 7장의 "정정 시간" 정의
    t_settle : 이후 이탈 없이 유지되는 정착 시각 [s]. 표준 정의
    overshoot : 최대 초과 각도 [deg]
    overshoot_pct : 계단 크기 대비 오버슈트 [%]
    rudder_travel : 타각 총 이동량 int|ddelta/dt|dt [deg]. HANDOFF 7장 "러더 사용량"
    rudder_effort : int delta^2 dt [deg^2*s]. 보조 지표
    delta_max_abs : 최대 타각 절댓값 [deg]
    sat_frac : 타각 포화 시간 비율 [-]
    band, ss_frac, step_size : 사용된 판정 조건 (재현성 기록용)

    Notes
    -----
    t_enter 와 t_settle 을 모두 낸다. HANDOFF 7장은 "±5% 이내 진입까지 소요
    시간"으로 정의하지만, 진입 후 다시 이탈하는 응답에서는 이 값이 성능을
    과대평가한다. 두 값이 크게 다르면 잔류 진동이 있다는 신호다.

    대역에 한 번도 못 들어오면 t_enter = nan, 끝까지 대역 밖이면 t_settle = nan.
    """

    rms_err_ss: float
    rms_err_all: float
    t_enter: float
    t_settle: float
    overshoot: float
    overshoot_pct: float
    rudder_travel: float
    rudder_effort: float
    delta_max_abs: float
    sat_frac: float
    band: float
    ss_frac: float
    step_size: float


def heading_metrics(
    out: SimResult,
    band: float = 0.05,
    ss_frac: float = 0.30,
    step_size: Optional[float] = None,
) -> HeadingMetrics:
    """계단 목표침로 응답에서 성능 지표를 계산한다.

    Parameters
    ----------
    band : 정정 시간 판정 대역, 계단 크기 대비 비율 (기본 0.05 = ±5%)
    ss_frac : 정상상태 구간 정의. 시뮬레이션 뒤쪽 비율 (기본 0.30 = 마지막 30%)
    step_size : 계단 크기 [rad]. None 이면 psi_ref 와 psi0 로부터 추정
    """
    if not 0.0 < band < 1.0:
        raise ValueError(f"band 는 (0, 1) 이어야 한다: {band}")
    if not 0.0 < ss_frac <= 1.0:
        raise ValueError(f"ss_frac 은 (0, 1] 이어야 한다: {ss_frac}")

    t = out.t
    err = out.err
    psi = out.psi
    ref = out.psi_ref
    dlt = out.delta
    n = len(t)

    # --- 계단 크기 ---
    if step_size is None:
        step_size = float(ssa(ref[-1] - psi[0]))
    amp = abs(step_size)

    # --- 침로 오차 RMS ---
    i_ss = max(0, int(n * (1.0 - ss_frac)))
    rms_err_ss = float(np.sqrt(np.mean(err[i_ss:] ** 2))) * R2D
    rms_err_all = float(np.sqrt(np.mean(err**2))) * R2D

    # --- 정정 시간 ---
    if amp < 1e-12:
        # 계단이 없으면 정정 시간과 오버슈트는 정의되지 않는다
        t_enter = math.nan
        t_settle = math.nan
        overshoot = 0.0
        overshoot_pct = 0.0
    else:
        tol = band * amp
        in_band = np.abs(err) <= tol

        enter_idx = np.flatnonzero(in_band)
        t_enter = float(t[enter_idx[0]]) if enter_idx.size else math.nan

        out_idx = np.flatnonzero(~in_band)
        if out_idx.size == 0:
            t_settle = float(t[0])
        elif out_idx[-1] == n - 1:
            t_settle = math.nan          # 끝까지 대역 밖 → 정착 실패
        else:
            t_settle = float(t[out_idx[-1] + 1])

        # --- 오버슈트 ---
        # 계단 방향 기준으로 부호를 맞춘 뒤 목표를 지나친 최대량
        excess = np.sign(step_size) * ssa(psi - ref)
        ov = float(max(0.0, np.max(excess)))
        overshoot = ov * R2D
        overshoot_pct = 100.0 * ov / amp

    # --- 러더 사용량 ---
    # int|ddelta/dt|dt = 타각이 실제로 움직인 총 거리.
    # 변화량의 절댓값 합으로 정확히 계산된다 (적분 근사 불필요).
    rudder_travel = float(np.sum(np.abs(np.diff(dlt)))) * R2D
    rudder_effort = float(np.trapezoid((dlt * R2D) ** 2, t))
    delta_max_abs = float(np.max(np.abs(dlt))) * R2D

    act = out.p.actuator
    if act.enable and np.isfinite(act.delta_max):
        sat_frac = float(np.mean(np.abs(dlt) >= act.delta_max * (1 - 1e-6)))
    else:
        sat_frac = 0.0

    return HeadingMetrics(
        rms_err_ss=rms_err_ss,
        rms_err_all=rms_err_all,
        t_enter=t_enter,
        t_settle=t_settle,
        overshoot=overshoot,
        overshoot_pct=overshoot_pct,
        rudder_travel=rudder_travel,
        rudder_effort=rudder_effort,
        delta_max_abs=delta_max_abs,
        sat_frac=sat_frac,
        band=band,
        ss_frac=ss_frac,
        step_size=step_size * R2D,
    )


_ROWS = [
    ("침로 오차 RMS (정상상태)", "rms_err_ss", "deg", "{:.4f}"),
    ("침로 오차 RMS (전 구간)", "rms_err_all", "deg", "{:.4f}"),
    ("대역 진입 시간", "t_enter", "s", "{:.2f}"),
    ("정착 시간", "t_settle", "s", "{:.2f}"),
    ("오버슈트", "overshoot", "deg", "{:.3f}"),
    ("오버슈트", "overshoot_pct", "%", "{:.2f}"),
    ("러더 사용량 int|d.delta|", "rudder_travel", "deg", "{:.2f}"),
    ("최대 타각", "delta_max_abs", "deg", "{:.2f}"),
    ("포화 시간 비율", "sat_frac", "-", "{:.3f}"),
]


def metrics_table(
    ms: Sequence[HeadingMetrics], labels: Optional[Sequence[str]] = None
) -> str:
    """지표를 markdown 비교표 문자열로 만든다.

    HANDOFF.md 8장 1항: 베이스라인 대비 비교표는 반드시 결과물에 포함한다.
    반환값을 docs/ 에 그대로 붙여 넣을 수 있다.
    """
    if isinstance(ms, HeadingMetrics):
        ms = [ms]
    if labels is None:
        labels = [f"case {i + 1}" for i in range(len(ms))]
    if len(labels) != len(ms):
        raise ValueError("labels 개수가 지표 개수와 다르다")

    lines = [
        "| 지표 | 단위 | " + " | ".join(labels) + " |",
        "|---|---|" + "---|" * len(ms),
    ]
    for title, attr, unit, fmt in _ROWS:
        cells = []
        for m in ms:
            v = getattr(m, attr)
            cells.append("n/a" if (isinstance(v, float) and math.isnan(v)) else fmt.format(v))
        lines.append(f"| {title} | {unit} | " + " | ".join(cells) + " |")

    ref = ms[0]
    lines.append("")
    lines.append(
        f"판정 조건: 대역 ±{ref.band * 100:.1f}% (계단 {ref.step_size:.1f} deg), "
        f"정상상태 구간 = 뒤쪽 {ref.ss_frac * 100:.0f}%"
    )
    return "\n".join(lines)
