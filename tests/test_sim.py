"""시뮬레이터 검증. C1~C6 은 HANDOFF.md 10장 4번의 계단 응답 확인 항목이다."""

import math

import numpy as np
import pytest

from usv_heading import SimOptions, nomoto_params, simulate_heading, ssa

D2R = math.pi / 180.0
R2D = 180.0 / math.pi


@pytest.fixture
def open_loop():
    """구동기 없는 개루프 계단 응답. delta = 5 deg."""
    p = nomoto_params("mariner").with_actuator(enable=False, delta_max=math.inf)
    delta = 5 * D2R
    out = simulate_heading(p, None, SimOptions(delta_open_loop=delta))
    return p, delta, out


# ---------------------------------------------------------------------------
# ssa
# ---------------------------------------------------------------------------

def test_ssa_basic():
    assert ssa(0.0) == pytest.approx(0.0)
    assert ssa(math.pi / 2) == pytest.approx(math.pi / 2)
    assert ssa(3 * math.pi) == pytest.approx(math.pi)


def test_ssa_wraps_across_180_boundary():
    """179 deg 와 -179 deg 의 차이는 358 deg 가 아니라 -2 deg 다."""
    err = ssa(179 * D2R - (-179 * D2R))
    assert err * R2D == pytest.approx(-2.0, abs=1e-9)


def test_ssa_range_is_half_open():
    """(-pi, pi] 를 유지한다. -pi 는 +pi 로 올라간다."""
    assert ssa(-math.pi) == pytest.approx(math.pi)
    assert ssa(math.pi) == pytest.approx(math.pi)

    xs = np.linspace(-20, 20, 4001)
    a = ssa(xs)
    assert np.all(a > -math.pi - 1e-12)
    assert np.all(a <= math.pi + 1e-12)
    # 원래 각도와 2pi 배수만큼만 차이나야 한다
    k = (xs - a) / (2 * math.pi)
    assert np.allclose(k, np.round(k), atol=1e-9)


def test_ssa_scalar_returns_scalar():
    assert isinstance(ssa(0.5), float)
    assert isinstance(ssa(np.array([0.5, 1.0])), np.ndarray)


# ---------------------------------------------------------------------------
# C1~C4: 개루프 계단 응답
# ---------------------------------------------------------------------------

def test_c1_matches_analytic_solution(open_loop):
    """C1: RK4 수치해가 해석해 r(t) = K*delta*(1 - e^(-t/T)) 와 일치."""
    p, delta, out = open_loop
    r_analytic = p.K * delta * (1 - np.exp(-out.t / p.T))
    rel_err = np.max(np.abs(out.r - r_analytic)) / np.max(np.abs(r_analytic))
    assert rel_err < 1e-6, f"상대오차 {rel_err:.3e}"


def test_c2_steady_state_gain(open_loop):
    """C2: 정상상태 선수각속도 = K * delta."""
    p, delta, out = open_loop
    expected = p.K * delta
    # t_end = 10T 이므로 지수 잔차는 e^(-10) = 4.5e-5
    assert out.r[-1] == pytest.approx(expected, rel=1e-3)


def test_c3_time_constant(open_loop):
    """C3: 63.2% 도달 시간 = T."""
    p, delta, out = open_loop
    r_ss = p.K * delta
    i63 = int(np.argmax(np.abs(out.r) >= 0.632 * abs(r_ss)))
    assert out.t[i63] == pytest.approx(abs(p.T), rel=0.02)


def test_c4_sign_convention(open_loop):
    """C4: delta > 0 -> r > 0 -> psi 증가."""
    p, delta, out = open_loop
    assert out.r[-1] > 0
    assert out.psi[-1] > 0
    assert np.all(np.diff(out.psi) >= 0)     # 단조 증가


def test_psi_matches_analytic_solution(open_loop):
    """선수각도 해석해와 일치해야 한다.

    r 을 적분하면  psi(t) = K*delta*(t - T*(1 - e^(-t/T))).
    C1 이 r 만 확인하므로, psidot = r 경로도 따로 확인한다.
    """
    p, delta, out = open_loop
    psi_analytic = p.K * delta * (out.t - p.T * (1 - np.exp(-out.t / p.T)))
    rel_err = np.max(np.abs(out.psi - psi_analytic)) / np.max(np.abs(psi_analytic))
    assert rel_err < 1e-6, f"상대오차 {rel_err:.3e}"


# ---------------------------------------------------------------------------
# C6: 구동기 제한
# ---------------------------------------------------------------------------

def test_c6a_rate_limit_respected():
    p = nomoto_params("mariner")
    opts = SimOptions(t_end=4 * p.T, delta_open_loop=2 * p.actuator.delta_max)
    out = simulate_heading(p, None, opts)
    rate = np.max(np.abs(np.diff(out.delta))) / out.opts.h
    assert rate <= p.actuator.delta_rate_max * (1 + 1e-3)


def test_c6b_saturation_respected():
    p = nomoto_params("mariner")
    opts = SimOptions(t_end=4 * p.T, delta_open_loop=2 * p.actuator.delta_max)
    out = simulate_heading(p, None, opts)
    assert np.max(np.abs(out.delta)) <= p.actuator.delta_max * (1 + 1e-9)


def test_actuator_reaches_commanded_angle_when_within_limits():
    p = nomoto_params("mariner")
    cmd = 10 * D2R
    out = simulate_heading(p, None, SimOptions(t_end=200.0, delta_open_loop=cmd))
    assert out.delta[-1] == pytest.approx(cmd, rel=1e-6)


# ---------------------------------------------------------------------------
# 적분기 성질
# ---------------------------------------------------------------------------

def test_rk4_convergence_order():
    """스텝을 반으로 줄이면 오차가 대략 16배 줄어야 한다 (4차 정확도).

    RK4 의 이론 차수는 4 이지만 배정밀도 바닥(~1e-16)에 가까워지면 반올림
    오차가 지배한다. 그래서 일부러 큰 스텝에서 비교한다.
    """
    p = nomoto_params("mariner").with_actuator(enable=False, delta_max=math.inf)
    delta = 5 * D2R

    def max_err(h):
        out = simulate_heading(
            p, None, SimOptions(t_end=2 * p.T, h=h, delta_open_loop=delta)
        )
        exact = p.K * delta * (1 - np.exp(-out.t / p.T))
        return np.max(np.abs(out.r - exact))

    e_coarse = max_err(p.T / 8)
    e_fine = max_err(p.T / 16)
    assert e_fine < e_coarse
    assert e_coarse / e_fine > 8      # 이론 16, 여유를 둬 8


def test_initial_conditions_are_honoured():
    p = nomoto_params("mariner")
    opts = SimOptions(
        t_end=50.0, psi0=0.5, r0=0.01, delta0=0.02, delta_open_loop=0.02
    )
    out = simulate_heading(p, None, opts)
    assert out.psi[0] == pytest.approx(0.5)
    assert out.r[0] == pytest.approx(0.01)
    assert out.delta[0] == pytest.approx(0.02)


def test_zero_input_stays_at_rest():
    p = nomoto_params("mariner")
    out = simulate_heading(p, None, SimOptions(t_end=500.0, delta_open_loop=0.0))
    assert np.allclose(out.psi, 0.0)
    assert np.allclose(out.r, 0.0)
    assert np.allclose(out.delta, 0.0)


def test_time_varying_reference_is_sampled():
    p = nomoto_params("mariner")
    out = simulate_heading(
        p, None, SimOptions(t_end=100.0, h=1.0, psi_ref=lambda t: 0.1 * t)
    )
    assert out.psi_ref[0] == pytest.approx(0.0)
    assert out.psi_ref[10] == pytest.approx(1.0)


def test_err_uses_ssa_not_raw_difference():
    """목표와 현재 침로가 ±180 도를 사이에 두고 있어도 오차가 작게 나와야 한다."""
    p = nomoto_params("mariner")
    out = simulate_heading(
        p,
        None,
        SimOptions(t_end=10.0, psi0=179 * D2R, psi_ref=-179 * D2R, delta_open_loop=0.0),
    )
    assert abs(out.err[0]) * R2D == pytest.approx(2.0, abs=1e-6)
