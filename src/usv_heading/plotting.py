"""침로 응답 플롯.

HANDOFF.md 8장 3항: 성능 지표가 좋아도 타각이 진동하면 채택하지 않는다.
따라서 타각 이력을 항상 함께 그린다.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence

import numpy as np

from .sim import SimResult

__all__ = ["plot_heading_response"]

R2D = 180.0 / math.pi


def plot_heading_response(
    outs: SimResult | Sequence[SimResult],
    labels: Optional[Sequence[str]] = None,
    title: str = "Heading response",
    savepath: Optional[str] = None,
):
    """침로 / 선수각속도 / 타각 3단 플롯. matplotlib Figure 를 반환한다."""
    import matplotlib.pyplot as plt

    if isinstance(outs, SimResult):
        outs = [outs]
    if labels is None:
        labels = [f"case {i + 1}" for i in range(len(outs))]

    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)

    # --- 침로 ---
    ax = axes[0]
    ax.plot(outs[0].t, outs[0].psi_ref * R2D, "k--", lw=1.2, label=r"$\psi_{ref}$")
    for o, lb in zip(outs, labels):
        ax.plot(o.t, o.psi * R2D, lw=1.5, label=lb)
    ax.set_ylabel(r"$\psi$ [deg]")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right")

    # --- 선수각속도 ---
    ax = axes[1]
    for o, lb in zip(outs, labels):
        ax.plot(o.t, o.r * R2D, lw=1.5, label=lb)
    ax.set_ylabel("r [deg/s]")
    ax.grid(True, alpha=0.3)

    # --- 타각 ---
    ax = axes[2]
    for o, lb in zip(outs, labels):
        ax.plot(o.t, o.delta * R2D, lw=1.5, label=lb)
    act = outs[0].p.actuator
    if act.enable and np.isfinite(act.delta_max):
        for sgn in (1, -1):
            ax.axhline(sgn * act.delta_max * R2D, color="r", ls=":", lw=1)
    ax.set_ylabel(r"$\delta$ [deg]")
    ax.set_xlabel("t [s]")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    if savepath:
        fig.savefig(savepath, dpi=130)
    return fig
