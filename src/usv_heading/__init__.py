"""usv-heading-rl — 무인수상정 침로 제어 PID 게인의 강화학습 최적화.

이 프로젝트는 AI 로 제어기를 대체하지 않는다.
최종 제어기는 검증 가능한 PID이며, 강화학습은 게인을 찾는 최적화 도구로만 쓴다.
자세한 내용은 HANDOFF.md 참조.
"""

from .control import PIDGains, PIDHeading
from .metrics import HeadingMetrics, heading_metrics, metrics_table
from .plant import (
    Actuator,
    NomotoParams,
    list_param_sets,
    nomoto_derivative,
    nomoto_params,
)
from .sim import SimOptions, SimResult, simulate_heading, ssa

__all__ = [
    "Actuator",
    "NomotoParams",
    "nomoto_params",
    "list_param_sets",
    "nomoto_derivative",
    "SimOptions",
    "SimResult",
    "simulate_heading",
    "ssa",
    "PIDGains",
    "PIDHeading",
    "HeadingMetrics",
    "heading_metrics",
    "metrics_table",
]

__version__ = "0.1.0"
