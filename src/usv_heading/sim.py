"""침로 제어 폐루프 시뮬레이션 (고정 스텝 RK4)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Optional, Protocol

import numpy as np

from .plant import NomotoParams, nomoto_derivative

__all__ = ["ssa", "SimOptions", "SimResult", "Controller", "simulate_heading"]


def ssa(angle):
    """Smallest signed angle. 각도를 (-pi, pi] 로 사상한다.

    침로 오차를 계산할 때 반드시 거쳐야 한다.
    예: psi_ref = 179 deg, psi = -179 deg 의 오차는 358 deg 가 아니라 -2 deg 다.
    """
    a = np.mod(np.asarray(angle, dtype=float) + np.pi, 2 * np.pi) - np.pi
    # mod 결과가 정확히 -pi 인 경우를 +pi 로 올려 (-pi, pi] 를 유지한다
    a = np.where(a == -np.pi, np.pi, a)
    return a if a.ndim else float(a)


class Controller(Protocol):
    """제어기 인터페이스.

    호출마다 (지령, 갱신된 내부상태) 를 돌려준다.
    state 는 첫 호출에서 None 으로 들어온다.
    """

    def __call__(
        self, t: float, psi: float, r: float, psi_ref: float, state, h: float
    ) -> tuple[float, object]:
        ...


ScalarOrFn = float | Callable[[float], float]


@dataclass
class SimOptions:
    """시뮬레이션 옵션.

    t_end, h 를 None 으로 두면 플랜트 시상수 T 로부터 자동 설정된다
    (t_end = 10|T|, h = |T|/500).

    t_end = 10|T| 인 이유: 계단 응답의 지수 잔차가 e^(-10) = 4.5e-5 로 떨어져
    "정상상태 r = K*delta" 를 1e-3 허용오차로 판정할 수 있다.
    6|T| 로 두면 잔차가 e^(-6) = 2.5e-3 라 판정이 실패한다 — 구현 오류가 아니라
    시뮬레이션이 짧은 것뿐이다. docs/journal.md 2026-08-07 항목 참조.
    """

    t_end: Optional[float] = None
    h: Optional[float] = None
    psi0: float = 0.0
    r0: float = 0.0
    delta0: float = 0.0
    psi_ref: ScalarOrFn = 0.0
    delta_open_loop: ScalarOrFn = 0.0

    def resolved(self, p: NomotoParams) -> "SimOptions":
        """T 기반 기본값을 채운 사본."""
        t_end = self.t_end if self.t_end is not None else 10.0 * abs(p.T)
        h = self.h if self.h is not None else abs(p.T) / 500.0
        return SimOptions(
            t_end=t_end,
            h=h,
            psi0=self.psi0,
            r0=self.r0,
            delta0=self.delta0,
            psi_ref=self.psi_ref,
            delta_open_loop=self.delta_open_loop,
        )


@dataclass
class SimResult:
    """시뮬레이션 결과. 모든 각도는 rad."""

    t: np.ndarray
    psi: np.ndarray
    r: np.ndarray
    delta: np.ndarray        # 실제 타각 (포화/변화율 제한 적용 후)
    delta_cmd: np.ndarray    # 제어기가 낸 지령
    psi_ref: np.ndarray
    err: np.ndarray          # ssa(psi_ref - psi)
    p: NomotoParams
    opts: SimOptions = field(repr=False)


def _as_fn(v: ScalarOrFn) -> Callable[[float], float]:
    if callable(v):
        return v
    return lambda _t: float(v)


def simulate_heading(
    p: NomotoParams,
    ctrl: Optional[Controller] = None,
    opts: Optional[SimOptions] = None,
) -> SimResult:
    """고정 스텝 RK4 로 폐루프(또는 개루프) 시뮬레이션을 돌린다.

    ctrl 이 None 이면 개루프이며 opts.delta_open_loop 가 지령으로 쓰인다.

    제어 지령은 스텝 구간 내에서 영차유지(zero-order hold)된다 — 실제 디지털
    제어기와 같다. RK4 의 중간 단계에서도 같은 지령을 쓴다.
    """
    opts = (opts or SimOptions()).resolved(p)
    h = opts.h
    n = int(opts.t_end / h) + 1

    t = np.arange(n) * h
    psi_log = np.zeros(n)
    r_log = np.zeros(n)
    delta_log = np.zeros(n)
    cmd_log = np.zeros(n)
    ref_log = np.zeros(n)

    x = np.array([opts.psi0, opts.r0, opts.delta0], dtype=float)
    state = None

    ref_fn = _as_fn(opts.psi_ref)
    ol_fn = _as_fn(opts.delta_open_loop)

    for k in range(n):
        tk = t[k]
        psi_ref = ref_fn(tk)

        if ctrl is None:
            delta_cmd = ol_fn(tk)
        else:
            delta_cmd, state = ctrl(tk, x[0], x[1], psi_ref, state, h)

        psi_log[k] = x[0]
        r_log[k] = x[1]
        cmd_log[k] = delta_cmd
        ref_log[k] = psi_ref
        delta_log[k] = _applied_delta(x, delta_cmd, p)

        if k == n - 1:
            break

        k1 = nomoto_derivative(x, delta_cmd, p)
        k2 = nomoto_derivative(x + (h / 2) * k1, delta_cmd, p)
        k3 = nomoto_derivative(x + (h / 2) * k2, delta_cmd, p)
        k4 = nomoto_derivative(x + h * k3, delta_cmd, p)
        x = x + (h / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

        if p.actuator.enable:
            x[2] = np.clip(x[2], -p.actuator.delta_max, p.actuator.delta_max)

    return SimResult(
        t=t,
        psi=psi_log,
        r=r_log,
        delta=delta_log,
        delta_cmd=cmd_log,
        psi_ref=ref_log,
        err=ssa(ref_log - psi_log),
        p=p,
        opts=opts,
    )


def _applied_delta(x: np.ndarray, delta_cmd: float, p: NomotoParams) -> float:
    if p.actuator.enable:
        return float(x[2])
    if np.isfinite(p.actuator.delta_max):
        return float(np.clip(delta_cmd, -p.actuator.delta_max, p.actuator.delta_max))
    return float(delta_cmd)
