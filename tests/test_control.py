"""PID 제어기 검증."""

import math

import numpy as np
import pytest

from usv_heading import (
    PIDGains,
    PIDHeading,
    SimOptions,
    nomoto_params,
    simulate_heading,
)

D2R = math.pi / 180.0
R2D = 180.0 / math.pi


@pytest.fixture
def plant():
    return nomoto_params("mariner")


def test_proportional_only_output():
    p = nomoto_params("mariner")
    ctrl = PIDHeading(PIDGains(Kp=2.0, Ki=0.0, Kd=0.0), p)
    u, _ = ctrl(0.0, psi=0.0, r=0.0, psi_ref=0.01, state=None, h=0.1)
    assert u == pytest.approx(2.0 * 0.01)


def test_derivative_acts_on_measurement_not_error():
    """미분항은 -r 에 걸린다. 목표가 계단으로 바뀌어도 미분 킥이 없다."""
    p = nomoto_params("mariner")
    ctrl = PIDHeading(PIDGains(Kp=0.0, Ki=0.0, Kd=5.0), p)

    # 오차가 아무리 커도 r = 0 이면 미분항은 0
    u, _ = ctrl(0.0, psi=0.0, r=0.0, psi_ref=1.0, state=None, h=0.1)
    assert u == pytest.approx(0.0)

    # r 이 있으면 그에 비례해 반대 방향
    u, _ = ctrl(0.0, psi=0.0, r=0.02, psi_ref=0.0, state=None, h=0.1)
    assert u == pytest.approx(-5.0 * 0.02)


def test_integral_accumulates():
    p = nomoto_params("mariner")
    ctrl = PIDHeading(PIDGains(Kp=0.0, Ki=1.0, Kd=0.0), p)
    state = None
    for _ in range(3):
        u, state = ctrl(0.0, psi=0.0, r=0.0, psi_ref=0.01, state=state, h=0.5)
    # 적분은 호출 후 갱신되므로 3회 호출 뒤 상태는 3*0.01*0.5
    assert state == pytest.approx(3 * 0.01 * 0.5)


def test_output_is_saturated_at_actuator_limit():
    p = nomoto_params("mariner")
    ctrl = PIDHeading(PIDGains(Kp=1000.0, Ki=0.0, Kd=0.0), p)
    u, _ = ctrl(0.0, psi=0.0, r=0.0, psi_ref=1.0, state=None, h=0.1)
    assert u == pytest.approx(p.actuator.delta_max)


def test_anti_windup_stops_integration_while_saturated():
    """포화 중이고 오차가 포화를 더 키우는 방향이면 적분이 멈춰야 한다."""
    p = nomoto_params("mariner")
    ctrl = PIDHeading(PIDGains(Kp=1000.0, Ki=1.0, Kd=0.0), p)

    state = 0.0
    for _ in range(10):
        _, state = ctrl(0.0, psi=0.0, r=0.0, psi_ref=1.0, state=state, h=0.5)
    assert state == pytest.approx(0.0), "포화 중 적분이 누적됐다"


def test_anti_windup_allows_integration_in_recovery_direction():
    """포화 중이라도 오차 부호가 반대면 적분은 계속돼야 한다 (탈출 가능)."""
    p = nomoto_params("mariner")
    ctrl = PIDHeading(PIDGains(Kp=1000.0, Ki=1.0, Kd=0.0), p)

    # 큰 양의 적분 상태에서 오차는 음수 → 지령은 여전히 양의 포화지만
    # 오차가 포화를 줄이는 방향이므로 적분이 진행돼야 한다
    _, state = ctrl(0.0, psi=0.0, r=0.0, psi_ref=-1e-6, state=100.0, h=0.5)
    assert state < 100.0


def test_no_windup_when_unsaturated():
    p = nomoto_params("mariner")
    ctrl = PIDHeading(PIDGains(Kp=0.1, Ki=1.0, Kd=0.0), p)
    _, state = ctrl(0.0, psi=0.0, r=0.0, psi_ref=0.001, state=0.0, h=0.5)
    assert state == pytest.approx(0.001 * 0.5)


def test_controller_uses_ssa_for_error():
    """±180 도를 사이에 둔 목표에서 짧은 쪽으로 돌아야 한다.

    psi = 179 deg 에서 psi_ref = -179 deg (= 181 deg) 로 가려면 +2 deg 만 더 돌면
    된다. ssa 를 안 거치면 -358 deg 로 계산되어 반대로 크게 돌아버린다.
    """
    p = nomoto_params("mariner")
    ctrl = PIDHeading(PIDGains(Kp=1.0, Ki=0.0, Kd=0.0), p)
    u, _ = ctrl(0.0, psi=179 * D2R, r=0.0, psi_ref=-179 * D2R, state=None, h=0.1)
    assert u * R2D == pytest.approx(+2.0, abs=1e-6)


# ---------------------------------------------------------------------------
# 폐루프 거동
# ---------------------------------------------------------------------------

def _closed_loop(gains, step_deg=1.0, t_end=None):
    p = nomoto_params("mariner")
    opts = SimOptions(t_end=t_end or 8 * p.T, psi_ref=step_deg * D2R)
    return simulate_heading(p, PIDHeading(gains, p), opts)


def test_closed_loop_pd_reduces_error():
    """PD 만으로도 오차가 초기값보다 크게 줄어야 한다.

    게인 값은 임의의 시험값이며 베이스라인 후보가 아니다 (HANDOFF 5장).
    여기서는 '제어 루프가 닫혀 있고 부호가 맞다'만 확인한다.
    """
    out = _closed_loop(PIDGains(Kp=3.0, Ki=0.0, Kd=200.0))
    assert abs(out.err[-1]) < 0.2 * abs(out.err[0])


def test_p_control_alone_has_no_steady_state_error():
    """침로 플랜트는 이미 적분기를 갖는다(type 1). 계단 목표에 Ki 가 필요 없다.

    psi(s)/delta(s) = K / (s(1+Ts)) 의 분모에 s 가 있으므로 P 제어만으로도
    계단 목표에 대한 정상상태 오차가 0 이다.

    베이스라인 튜닝 시 유의할 점이다 — 정상상태 오차를 없애려고 Ki 를 키우면
    극점이 하나 더 늘어 오버슈트와 진동만 커진다.
    """
    g = PIDGains(Kp=3.0, Ki=0.0, Kd=200.0)
    p = nomoto_params("mariner")

    short = _closed_loop(g, t_end=8 * p.T)
    long = _closed_loop(g, t_end=24 * p.T)

    assert abs(short.err[-1]) * R2D < 1e-4

    # 남은 오차가 정상상태 편차가 아니라 감쇠 중인 과도응답임을 확인한다.
    # 정상상태 편차라면 시뮬레이션을 늘려도 줄지 않는다.
    assert abs(long.err[-1]) < abs(short.err[-1]) / 100


def test_integral_rejects_constant_disturbance():
    """Ki 의 역할은 계단 추종이 아니라 상수 외란 제거다.

    조류·바람 같은 상수 외란을 타각 편향으로 흉내 낸다.
    Ki = 0 이면 정상상태 오차가 남고, Ki > 0 이면 0 으로 간다.
    3단계(외란 강건성)에서 Ki 가 왜 필요한지에 대한 근거가 된다.
    """
    p = nomoto_params("mariner")
    bias = 0.5 * D2R          # 상수 타각 편향

    def run(Ki):
        pid = PIDHeading(PIDGains(Kp=3.0, Ki=Ki, Kd=200.0), p)

        def ctrl(t, psi, r, psi_ref, state, h):
            u, state = pid(t, psi, r, psi_ref, state, h)
            return u + bias, state

        out = simulate_heading(p, ctrl, SimOptions(t_end=20 * p.T, psi_ref=0.0))
        return abs(out.err[-1]) * R2D

    err_no_integral = run(0.0)
    err_with_integral = run(0.02)

    assert err_no_integral > 0.1, "외란이 실제로 오차를 만들어야 테스트가 의미 있다"
    assert err_with_integral < 1e-4
    assert err_with_integral < err_no_integral / 100


def test_closed_loop_negative_step_is_mirror_image():
    """플랜트가 선형이고 포화에 걸리지 않으면 응답이 대칭이어야 한다."""
    g = PIDGains(Kp=3.0, Ki=0.0, Kd=200.0)
    pos = _closed_loop(g, step_deg=0.5)
    neg = _closed_loop(g, step_deg=-0.5)
    assert np.max(np.abs(pos.psi + neg.psi)) < 1e-9
    assert np.max(np.abs(pos.delta + neg.delta)) < 1e-9


def test_zero_gains_leave_rudder_at_zero():
    out = _closed_loop(PIDGains(Kp=0.0, Ki=0.0, Kd=0.0))
    assert np.allclose(out.delta, 0.0)
    assert np.allclose(out.psi, 0.0)
