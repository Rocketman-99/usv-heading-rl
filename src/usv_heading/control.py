"""침로 유지 PID 제어기.

⚠ 게인 값은 이 모듈이 정하지 않는다. 반드시 호출자가 주입한다.
  베이스라인 게인은 작업자(David)가 수동 튜닝한다 — HANDOFF.md 5장, 9장.
  이 파일에 자동 튜닝(pidtune / 최적화 / 격자탐색)을 추가하지 말 것.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .plant import NomotoParams
from .sim import ssa

__all__ = ["PIDGains", "PIDHeading"]


@dataclass(frozen=True)
class PIDGains:
    """PID 게인.

    단위: delta_cmd[rad] = Kp * e[rad] + Ki * int(e)[rad*s] + Kd * (-r)[rad/s]
    """

    Kp: float
    Ki: float
    Kd: float

    def __str__(self) -> str:
        return f"Kp={self.Kp:.4g} Ki={self.Ki:.4g} Kd={self.Kd:.4g}"


class PIDHeading:
    """PID 침로 제어기.

    제어칙::

        e        = ssa(psi_ref - psi)
        delta_cmd = Kp*e + Ki*integral(e) + Kd*(-r)

    미분항에 e 의 수치미분이 아니라 -r 을 쓴다 (derivative on measurement).
    목표 침로가 계단으로 바뀔 때 미분 킥이 생기지 않고, r 은 상태로 직접
    측정되므로 수치미분 잡음도 없다.

    적분 안티와인드업은 conditional integration 방식이다. 지령이 포화 중이고
    오차가 포화를 더 키우는 방향일 때만 적분을 멈춘다.

    simulate_heading 에 그대로 넘길 수 있다::

        ctrl = PIDHeading(gains, p)
        out = simulate_heading(p, ctrl, opts)
    """

    def __init__(self, gains: PIDGains, p: NomotoParams):
        self.gains = gains
        self.p = p
        if p.actuator.enable and np.isfinite(p.actuator.delta_max):
            self.u_max = p.actuator.delta_max
        else:
            self.u_max = np.inf

    def __call__(
        self,
        t: float,
        psi: float,
        r: float,
        psi_ref: float,
        state: Optional[float],
        h: float,
    ) -> tuple[float, float]:
        e_int = 0.0 if state is None else float(state)
        g = self.gains

        e = ssa(psi_ref - psi)

        u_unsat = g.Kp * e + g.Ki * e_int + g.Kd * (-r)
        u_sat = float(np.clip(u_unsat, -self.u_max, self.u_max))

        # 포화 중이면서 오차가 포화를 더 키우는 방향일 때만 적분 정지
        saturated = u_unsat != u_sat
        winding = saturated and (np.sign(e) == np.sign(u_unsat))
        if not winding:
            e_int = e_int + e * h

        return u_sat, e_int
