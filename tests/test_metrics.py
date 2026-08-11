"""지표 계산 검증.

지표는 합성 응답(직접 만든 SimResult)으로 검증한다. 시뮬레이션을 거치면
"지표가 틀렸는지 시뮬레이터가 틀렸는지" 구분이 안 되기 때문이다.
"""

import math

import numpy as np
import pytest

from usv_heading import (
    HeadingMetrics,
    SimOptions,
    heading_metrics,
    metrics_table,
    nomoto_params,
)
from usv_heading.sim import SimResult, ssa

D2R = math.pi / 180.0
R2D = 180.0 / math.pi


def make_result(psi_deg, ref_deg=10.0, delta_deg=None, h=0.1, actuator=True):
    """합성 SimResult 를 만든다. psi_deg 는 시계열, ref 는 상수 계단."""
    psi = np.asarray(psi_deg, dtype=float) * D2R
    n = len(psi)
    t = np.arange(n) * h
    ref = np.full(n, ref_deg * D2R)
    delta = (
        np.zeros(n) if delta_deg is None else np.asarray(delta_deg, dtype=float) * D2R
    )
    p = nomoto_params("mariner")
    if not actuator:
        p = p.with_actuator(enable=False, delta_max=math.inf)
    return SimResult(
        t=t,
        psi=psi,
        r=np.gradient(psi, h),
        delta=delta,
        delta_cmd=delta,
        psi_ref=ref,
        err=ssa(ref - psi),
        p=p,
        opts=SimOptions().resolved(p),
    )


# ---------------------------------------------------------------------------
# 기본 지표
# ---------------------------------------------------------------------------

def test_perfect_tracking_gives_zero_error():
    out = make_result(np.full(100, 10.0), ref_deg=10.0)
    m = heading_metrics(out)
    assert m.rms_err_ss == pytest.approx(0.0)
    assert m.rms_err_all == pytest.approx(0.0)
    assert m.overshoot == pytest.approx(0.0)


def test_step_size_inferred_from_reference_and_initial_heading():
    out = make_result([0.0] + [10.0] * 99, ref_deg=10.0)
    m = heading_metrics(out)
    assert m.step_size == pytest.approx(10.0)


def test_overshoot_measured_beyond_target():
    # 12 deg 까지 올라갔다가 10 deg 로 안착 → 오버슈트 2 deg = 20%
    psi = [0.0] + [12.0] * 5 + [10.0] * 94
    m = heading_metrics(make_result(psi, ref_deg=10.0))
    assert m.overshoot == pytest.approx(2.0)
    assert m.overshoot_pct == pytest.approx(20.0)


def test_no_overshoot_when_response_stays_below_target():
    psi = [0.0] + list(np.linspace(0, 10, 99))
    m = heading_metrics(make_result(psi, ref_deg=10.0))
    assert m.overshoot == pytest.approx(0.0, abs=1e-12)


def test_overshoot_for_negative_step():
    """음의 계단에서도 '목표를 지나친 양'이 양수로 나와야 한다."""
    psi = [0.0] + [-12.0] * 5 + [-10.0] * 94
    m = heading_metrics(make_result(psi, ref_deg=-10.0))
    assert m.overshoot == pytest.approx(2.0)


def test_rudder_travel_is_total_path_length():
    """0 -> 5 -> 0 -> 5 이면 총 이동량은 15 deg."""
    delta = [0.0, 5.0, 0.0, 5.0]
    m = heading_metrics(make_result([10.0] * 4, delta_deg=delta))
    assert m.rudder_travel == pytest.approx(15.0)


def test_delta_max_abs_and_sat_frac():
    p = nomoto_params("mariner")
    dmax = p.actuator.delta_max * R2D
    delta = [0.0, dmax, dmax, 0.0]
    m = heading_metrics(make_result([10.0] * 4, delta_deg=delta))
    assert m.delta_max_abs == pytest.approx(dmax)
    assert m.sat_frac == pytest.approx(0.5)


def test_sat_frac_zero_when_actuator_disabled():
    m = heading_metrics(make_result([10.0] * 4, delta_deg=[0, 1000, 0, 0], actuator=False))
    assert m.sat_frac == 0.0


# ---------------------------------------------------------------------------
# 정정 시간 — t_enter vs t_settle
# ---------------------------------------------------------------------------

def test_t_enter_and_t_settle_agree_for_clean_response():
    """한 번 들어가서 안 나오면 두 값이 같다."""
    psi = [0.0] * 10 + [10.0] * 90
    m = heading_metrics(make_result(psi, ref_deg=10.0), band=0.05)
    assert m.t_enter == pytest.approx(1.0)
    assert m.t_settle == pytest.approx(1.0)


def test_t_settle_later_than_t_enter_when_response_re_exits_band():
    """진입 후 다시 이탈하면 t_enter 는 성능을 과대평가한다.

    HANDOFF 7장 정의(t_enter)만으로는 잔류 진동을 놓친다는 것이 이 테스트의 요점이다.
    """
    # 0 s 에 진입 -> 다시 이탈(2 deg 오차) -> 5 s 이후 재진입해 유지.
    # psi[0] 이 목표와 같아 계단 크기를 추정할 수 없으므로 명시한다.
    psi = [10.0] + [12.0] * 49 + [10.0] * 50
    m = heading_metrics(
        make_result(psi, ref_deg=10.0), band=0.05, step_size=10.0 * D2R
    )
    assert m.t_enter == pytest.approx(0.0)
    assert m.t_settle == pytest.approx(5.0)
    assert m.t_settle > m.t_enter


def test_t_enter_nan_when_band_never_reached():
    psi = [0.0] * 100          # 목표 10 deg 에 전혀 접근 못 함
    m = heading_metrics(make_result(psi, ref_deg=10.0))
    assert math.isnan(m.t_enter)
    assert math.isnan(m.t_settle)


def test_t_settle_nan_when_diverging_at_end():
    """끝에서 대역 밖이면 정착 실패."""
    psi = [10.0] * 90 + list(np.linspace(10, 40, 10))
    m = heading_metrics(make_result(psi, ref_deg=10.0))
    assert math.isnan(m.t_settle)


def test_band_width_changes_settling_time():
    psi = [0.0] * 10 + [10.4] * 10 + [10.0] * 80
    loose = heading_metrics(make_result(psi, ref_deg=10.0), band=0.05)   # ±0.5 deg
    tight = heading_metrics(make_result(psi, ref_deg=10.0), band=0.01)   # ±0.1 deg
    assert loose.t_enter < tight.t_enter


# ---------------------------------------------------------------------------
# 정상상태 구간
# ---------------------------------------------------------------------------

def test_ss_window_ignores_transient():
    """전 구간 RMS 는 과도응답에 오염되지만 정상상태 RMS 는 아니다."""
    psi = [0.0] * 50 + [10.0] * 50
    m = heading_metrics(make_result(psi, ref_deg=10.0), ss_frac=0.3)
    assert m.rms_err_ss == pytest.approx(0.0)
    assert m.rms_err_all > 5.0


def test_ss_frac_one_equals_full_run():
    psi = list(np.linspace(0, 10, 100))
    m = heading_metrics(make_result(psi, ref_deg=10.0), ss_frac=1.0)
    assert m.rms_err_ss == pytest.approx(m.rms_err_all)


# ---------------------------------------------------------------------------
# 입력 검증 / 엣지케이스
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("band", [0.0, 1.0, -0.1, 1.5])
def test_invalid_band_raises(band):
    with pytest.raises(ValueError, match="band"):
        heading_metrics(make_result([10.0] * 10), band=band)


@pytest.mark.parametrize("ss_frac", [0.0, 1.5, -0.2])
def test_invalid_ss_frac_raises(ss_frac):
    with pytest.raises(ValueError, match="ss_frac"):
        heading_metrics(make_result([10.0] * 10), ss_frac=ss_frac)


def test_zero_step_does_not_crash():
    """계단이 없으면 정정 시간과 오버슈트는 정의되지 않는다."""
    m = heading_metrics(make_result([0.0] * 50, ref_deg=0.0))
    assert math.isnan(m.t_enter)
    assert m.overshoot == 0.0
    assert m.rms_err_ss == pytest.approx(0.0)


def test_error_uses_ssa_across_180():
    """179 -> -179 를 358 deg 오차로 세면 안 된다."""
    m = heading_metrics(make_result([179.0] * 50, ref_deg=-179.0, delta_deg=None))
    assert m.rms_err_ss == pytest.approx(2.0, abs=1e-6)


# ---------------------------------------------------------------------------
# 비교표
# ---------------------------------------------------------------------------

def test_metrics_table_has_one_column_per_case():
    ms = [heading_metrics(make_result([10.0] * 50)) for _ in range(2)]
    tbl = metrics_table(ms, ["baseline", "RL"])
    header = tbl.splitlines()[0]
    assert "baseline" in header and "RL" in header
    assert header.count("|") == 5      # 지표 | 단위 | a | b |


def test_metrics_table_accepts_single_metrics():
    tbl = metrics_table([heading_metrics(make_result([10.0] * 50))])
    assert "침로 오차 RMS" in tbl
    assert "판정 조건" in tbl


def test_metrics_table_renders_nan_as_na():
    m = heading_metrics(make_result([0.0] * 50, ref_deg=10.0))
    assert "n/a" in metrics_table([m])


def test_metrics_table_label_count_mismatch_raises():
    ms = [heading_metrics(make_result([10.0] * 50))]
    with pytest.raises(ValueError, match="labels"):
        metrics_table(ms, ["a", "b"])
