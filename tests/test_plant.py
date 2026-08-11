"""플랜트 파라미터와 동역학 검증.

여기 있는 테스트 두 개(재유도 검증)는 단순한 회귀 테스트가 아니다.
HANDOFF.md 9장 "문헌 계수를 출처 없이 사용하는 것" 금지에 대한 실행 가능한 대응이다.
문헌값을 옮겨 적기만 하면 오타를 잡을 수 없으므로, 원 미계수로부터 다시 계산해
일치하는지를 CI 가 매번 확인한다.
"""

import math

import numpy as np
import pytest

from usv_heading import list_param_sets, nomoto_derivative, nomoto_params


# ---------------------------------------------------------------------------
# 문헌값 재유도
# ---------------------------------------------------------------------------

def test_mariner_k_t_rederived_from_hydrodynamic_derivatives():
    """Mariner 급 K, T 를 선형 유체력 미계수에서 직접 재유도해 문헌값과 대조한다.

    출처: MSS CRAFT/SHIP/models/mariner.m 의 무차원 선형 미계수
          (원 출처 Chislett & Stroem-Tejsen 1965)
    대조: MSS mssExamples/exNomoto.m 의 K = 0.185, T1/T2/T3 = 118/7.8/18.5

    선형 sway-yaw 방정식 (prime system, 무차원 시간 t' = t*U/L):
        [m22 m23] [v'dot]   [Yv Yr] [v']   [Yd]
        [m32 m33] [r'dot] = [Nv Nr] [r'] + [Nd] * delta
    """
    L, U = 160.93, 7.7175
    m, Iz, xG = 798e-5, 39.2e-5, -0.023
    Yvdot, Nvdot = -748e-5, 4.646e-5
    Yrdot, Nrdot = -9.354e-5, -43.8e-5
    Yv, Nv = -1160e-5, -264e-5
    Yr, Nr = -499e-5, -166e-5
    Yd, Nd = 278e-5, -139e-5

    m22 = m - Yvdot
    m23 = m * xG - Yrdot
    m32 = m * xG - Nvdot
    m33 = Iz - Nrdot

    M = np.array([[m22, m23], [m32, m33]])
    N = -np.array([[Yv, Yr], [Nv, Nr]])
    b = np.array([Yd, Nd])

    # (M s + N) x = b * delta 에서 r'(s)/delta(s) 를 Cramer 로 구한다
    a2 = np.linalg.det(M)
    a1 = m22 * N[1, 1] + m33 * N[0, 0] - m23 * N[1, 0] - m32 * N[0, 1]
    a0 = np.linalg.det(N)
    n1 = m22 * b[1] - m32 * b[0]
    n0 = N[0, 0] * b[1] - N[1, 0] * b[0]

    K_prime = n0 / a0
    T3_prime = n1 / n0
    T1_prime, T2_prime = -1.0 / np.roots([a2, a1, a0])

    # 차원 복원: K = K' * U/L,  T_i = T_i' * L/U
    scale = L / U
    K = abs(K_prime * U / L)
    T1, T2 = sorted([T1_prime * scale, T2_prime * scale], reverse=True)
    T3 = T3_prime * scale

    # exNomoto.m 기재값과 대조 (반올림 자릿수 범위)
    assert K == pytest.approx(0.185, abs=5e-4)
    assert T1 == pytest.approx(118.0, abs=0.5)
    assert T2 == pytest.approx(7.8, abs=0.5)
    assert T3 == pytest.approx(18.5, abs=0.5)

    # 1차 등가 시상수가 파라미터 세트의 T 와 일치하는지
    p = nomoto_params("mariner")
    assert (T1 + T2 - T3) == pytest.approx(p.T, abs=0.5)
    assert p.K == pytest.approx(K, abs=5e-4)


def test_otter_yaw_inertia_recomputed_from_mss():
    """Otter USV 의 M(6,6) 을 MSS otter.m 의 구성대로 재계산한다.

    출처: MSS CRAFT/USV/models/otter.m (mp = 25 kg, rp = [0,0,0])
        Ig  = Ig_CG - m*S(r_hull)^2 - mp*S(r_payload)^2
        MRB = H(rg)' * MRB_CG * H(rg),  H(r) = [[I, S(r)'], [0, I]]
        MA(6,6) = 1.7 * Ig(3,3)
    """
    def S(r):
        return np.array([[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]])

    m, mp, L, B = 55.0, 25.0, 2.0, 1.08
    rg_hull = np.array([0.2, 0.0, -0.2])
    rp = np.zeros(3)
    R44, R55, R66 = 0.4 * B, 0.25 * L, 0.25 * L

    Ig_CG = m * np.diag([R44**2, R55**2, R66**2])
    rg = (m * rg_hull + mp * rp) / (m + mp)
    Ig = Ig_CG - m * S(rg_hull - rg) @ S(rg_hull - rg) - mp * S(rp - rg) @ S(rp - rg)

    H = np.block([[np.eye(3), S(rg).T], [np.zeros((3, 3)), np.eye(3)]])
    MRB_CG = np.block(
        [[(m + mp) * np.eye(3), np.zeros((3, 3))], [np.zeros((3, 3)), Ig]]
    )
    M66 = (H.T @ MRB_CG @ H)[5, 5] + 1.7 * Ig[2, 2]

    assert M66 == pytest.approx(40.49375, rel=1e-9)
    assert nomoto_params("otter").K == pytest.approx(1.0 / M66, rel=1e-12)


# ---------------------------------------------------------------------------
# 파라미터 세트 위생
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("p", list_param_sets(), ids=lambda p: p.name)
def test_every_param_set_cites_a_source(p):
    """HANDOFF.md 9장: 문헌 계수를 출처 없이 사용하지 않는다.

    새 파라미터 세트를 추가할 때 출처를 빠뜨리면 CI 가 막는다.
    """
    assert p.source.strip(), f"{p.name}: source 가 비어 있다"
    assert len(p.source) > 40, f"{p.name}: source 가 너무 짧다 — 실제 출처인지 확인"
    assert p.input_type in ("rudder_angle", "yaw_moment")
    assert math.isfinite(p.K) and p.K != 0
    assert math.isfinite(p.T) and p.T != 0


def test_unknown_param_set_raises():
    with pytest.raises(KeyError, match="알 수 없는"):
        nomoto_params("nonexistent_ship")


def test_with_actuator_returns_copy():
    p = nomoto_params("mariner")
    q = p.with_actuator(enable=False)
    assert p.actuator.enable is True      # 원본 불변
    assert q.actuator.enable is False
    assert q.K == p.K and q.T == p.T


# ---------------------------------------------------------------------------
# 상태미분
# ---------------------------------------------------------------------------

def test_derivative_psidot_equals_r():
    p = nomoto_params("mariner")
    x = np.array([0.3, 0.02, 0.1])
    assert nomoto_derivative(x, 0.1, p)[0] == pytest.approx(0.02)


def test_derivative_equilibrium_at_steady_turn():
    """r = K*delta 이면 rdot = 0 이어야 한다."""
    p = nomoto_params("mariner").with_actuator(enable=False)
    delta = math.radians(3.0)
    x = np.array([0.0, p.K * delta, delta])
    assert nomoto_derivative(x, delta, p)[1] == pytest.approx(0.0, abs=1e-15)


def test_derivative_rate_limit_applied():
    p = nomoto_params("mariner")
    x = np.array([0.0, 0.0, 0.0])
    # 큰 지령이라도 변화율 한계를 넘지 않는다
    d = nomoto_derivative(x, p.actuator.delta_max, p)
    assert d[2] == pytest.approx(p.actuator.delta_rate_max)


def test_derivative_blocks_pushing_past_saturation():
    """포화 경계에서 바깥으로 미는 미분은 0 이어야 한다 (안티와인드업)."""
    p = nomoto_params("mariner")
    x = np.array([0.0, 0.0, p.actuator.delta_max])
    assert nomoto_derivative(x, 2 * p.actuator.delta_max, p)[2] == 0.0

    # 반대 방향으로는 움직일 수 있어야 한다
    assert nomoto_derivative(x, -p.actuator.delta_max, p)[2] < 0


def test_derivative_actuator_disabled_uses_command_directly():
    p = nomoto_params("mariner").with_actuator(enable=False)
    delta = math.radians(5.0)
    x = np.array([0.0, 0.0, 0.0])     # 상태의 delta 는 0 이지만
    d = nomoto_derivative(x, delta, p)
    assert d[1] == pytest.approx(p.K * delta / p.T)   # 지령이 바로 반영
    assert d[2] == 0.0
