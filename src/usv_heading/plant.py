"""1차 Nomoto 플랜트 모델과 문헌 출처가 확인된 파라미터 세트.

모델:
    T * rdot + r = K * delta
    psidot = r

파라미터 출처는 docs/02_plant_model.md 에 정리되어 있다.
HANDOFF.md 9장: 문헌 계수를 출처 없이 사용하지 않는다.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import inf, radians

import numpy as np

__all__ = ["Actuator", "NomotoParams", "nomoto_params", "list_param_sets", "nomoto_derivative"]


@dataclass(frozen=True)
class Actuator:
    """타기(구동기) 제한.

    순수 Nomoto 에는 없는 요소다. 구동기 한계가 없으면 학습이 무한대 타각률을
    쓰는 해를 찾아내므로 "러더 사용량" 지표가 의미를 잃는다.
    포함 여부는 docs/decisions.md D-006 (OPEN).
    """

    enable: bool = True
    delta_max: float = radians(40.0)       # 최대 타각 [rad]
    delta_rate_max: float = radians(5.0)   # 최대 타각 변화율 [rad/s]
    t_delta: float = 1.0                   # 타기 1차 지연 시상수 [s]


@dataclass(frozen=True)
class NomotoParams:
    """1차 Nomoto 파라미터.

    Attributes
    ----------
    name : 세트 이름
    K : Nomoto 이득. input_type='rudder_angle' 이면 [1/s]
    T : Nomoto 시상수 [s]
    input_type : 'rudder_angle' (타각 delta) 또는 'yaw_moment' (요 모멘트 tau_N)
    k_unit : K 의 물리 단위 문자열
    L, U : 기준 선체 길이 [m], 기준 속력 [m/s]
    T1, T2, T3 : 2차 Nomoto 시상수 [s] (있는 경우)
    actuator : 구동기 제한
    source : 출처 문자열. 비어 있으면 안 된다
    notes : 사용 시 주의사항
    """

    name: str
    K: float
    T: float
    input_type: str
    k_unit: str
    L: float
    U: float
    actuator: Actuator
    source: str
    notes: str = ""
    T1: float = float("nan")
    T2: float = float("nan")
    T3: float = float("nan")

    def with_actuator(self, **kwargs) -> "NomotoParams":
        """구동기 설정만 바꾼 사본을 만든다. 예: p.with_actuator(enable=False)"""
        return replace(self, actuator=replace(self.actuator, **kwargs))


# ---------------------------------------------------------------------------
# 파라미터 세트 (출처 명시 필수)
# ---------------------------------------------------------------------------

_MARINER = NomotoParams(
    name="mariner",
    K=0.185,
    T=118.0 + 7.8 - 18.5,          # = 107.3 s
    T1=118.0,
    T2=7.8,
    T3=18.5,
    input_type="rudder_angle",
    k_unit="(deg/s)/deg = 1/s",
    L=160.93,
    U=7.7175,                       # 15 knots
    actuator=Actuator(
        enable=True,
        delta_max=radians(40.0),
        delta_rate_max=radians(5.0),
        t_delta=1.0,
    ),
    source=(
        "K, T1, T2, T3: Fossen, MSS (Marine Systems Simulator), "
        "mssExamples/exNomoto.m, github.com/cybergalactic/MSS (2026-08-07 확인). "
        "원 출처: M. S. Chislett and J. Stroem-Tejsen (1965), 'Planar Motion "
        "Mechanism Tests and Full-Scale Steering and Manoeuvring Predictions for "
        "a Mariner Class Vessel', International Shipbuilding Progress 12(129), "
        "201-224 (doi:10.3233/ISP-1965-1212902); 동 내용의 기관 보고서는 Report Hy-6, "
        "Hydro- and Aerodynamics Laboratory, Lyngby, Denmark. "
        "(⚠ MSS mariner.m 헤더는 이를 'Hy-5' 로 적고 있으나 Hy-5 는 "
        "Abkowitz (1964), 'Lectures on Ship Hydrodynamics - Steering and "
        "Manoeuvrability' 이다. 2026-08-15 확인.) "
        "| L, U0 및 구동기 한계(40 deg / 5 deg/s): "
        "MSS CRAFT/SHIP/models/mariner.m (IMO 35 deg rudder execute 대응). | "
        "K, T 값은 동 파일의 선형 유체력 미계수에서 r(s)/delta(s) 를 직접 "
        "재유도해 일치 확인함 (K=0.18499, T=107.21 s). "
        "테스트: tests/test_plant.py::test_mariner_k_t_rederived_from_hydrodynamic_derivatives"
    ),
    notes=(
        "L = 160.93 m 화물선. 이 프로젝트의 대상(USV)과 규모가 다르다 — D-004. "
        "침로 안정(K > 0, T > 0). "
        "선형 근사는 작은 타각에서만 유효하다: 비선형 모델 대비 실효 이득이 "
        "delta=5 deg 에서 42%, 35 deg 에서 9%. docs/02_plant_model.md 참조."
    ),
)

_TANKER = NomotoParams(
    name="tanker",
    K=-0.019,
    T=-124.1 + 16.4 - 46.0,        # = -153.7 s
    T1=-124.1,
    T2=16.4,
    T3=46.0,
    input_type="rudder_angle",
    k_unit="(deg/s)/deg = 1/s",
    L=float("nan"),
    U=float("nan"),
    actuator=Actuator(
        enable=True,
        delta_max=radians(35.0),
        delta_rate_max=radians(3.0),
        t_delta=1.0,
    ),
    source=(
        "Fossen, MSS, mssExamples/exNomoto.m ('Oil tanker'), "
        "github.com/cybergalactic/MSS (2026-08-07 확인). "
        "⚠ L, U, 구동기 한계는 MSS 에 명시되어 있지 않아 통상값을 가정했다. "
        "사용하려면 출처를 보완해야 한다."
    ),
    notes=(
        "T < 0 이므로 개루프 침로 불안정선. 1단계 대상이 아니다. "
        "3단계 이후 게인 강건성 비교용 대조군 후보."
    ),
)

# MSS SIMotter.m 은 요 모멘트 tau_N 입력의 Nomoto 형태를 쓴다: K = T / M(6,6).
# M(6,6) = 40.49375 kg*m^2 (mp = 25 kg, rp = [0,0,0]) 로 재계산해 확인.
# 테스트: tests/test_plant.py::test_otter_yaw_inertia_recomputed_from_mss
_OTTER_M66 = 40.49375

_OTTER = NomotoParams(
    name="otter",
    K=1.0 / _OTTER_M66,             # = 0.024695
    T=1.0,
    input_type="yaw_moment",
    k_unit="(rad/s)/(N*m) = 1/(kg*m^2)",
    L=2.0,
    U=6 * 0.5144,                   # 6 knots
    actuator=Actuator(enable=False, delta_max=inf, delta_rate_max=inf, t_delta=1.0),
    source=(
        "T = 1 s 및 K = T / M(6,6): Fossen, MSS, CRAFT/USV/SIMotter.m (97-99행), "
        "github.com/cybergalactic/MSS (2026-08-07 확인). "
        "M(6,6) = 40.49375 kg*m^2 는 CRAFT/USV/models/otter.m 의 MRB + MA 구성을 "
        "mp = 25 kg, rp = [0,0,0] 조건으로 재계산해 확인. "
        "주요 제원 L = 2.0 m, B = 1.08 m, m = 55 kg: 동 파일."
    ),
    notes=(
        "⚠ 입력이 타각이 아니라 요 모멘트다. Otter 는 차동추진 USV 라 물리적 러더가 "
        "없다. 이 세트를 쓰려면 '러더 사용량' 지표를 모멘트 기준으로 재정의해야 한다. "
        "⚠ T = 1 s 는 문헌 동정값이 아니라 MSS 오토파일럿 설계에서 가정한 값이다 "
        "(K 를 T 로부터 역산). 실험 기반 계수가 아니다."
    ),
)

_SETS = {p.name: p for p in (_MARINER, _TANKER, _OTTER)}

#: 기본 세트. 어떤 선박을 이 프로젝트의 대상으로 할지는 아직 미결정이다
#: (docs/decisions.md D-004). 'mariner' 는 "K, T 가 1차 Nomoto 형태로 문헌에
#: 직접 제시되고 재유도까지 확인된 유일한 세트"라는 이유로 임시 지정한 것이다.
DEFAULT_SET = "mariner"


def nomoto_params(name: str = DEFAULT_SET) -> NomotoParams:
    """이름으로 파라미터 세트를 가져온다."""
    key = name.lower()
    if key not in _SETS:
        raise KeyError(
            f"알 수 없는 파라미터 세트 {name!r}. "
            f"사용 가능: {sorted(_SETS)}"
        )
    return _SETS[key]


def list_param_sets() -> list[NomotoParams]:
    """등록된 모든 파라미터 세트."""
    return list(_SETS.values())


# ---------------------------------------------------------------------------
# 동역학
# ---------------------------------------------------------------------------

def nomoto_derivative(x: np.ndarray, delta_cmd: float, p: NomotoParams) -> np.ndarray:
    """1차 Nomoto + 구동기 동특성의 상태미분.

    상태 x = [psi, r, delta]:
        psi   선수각 [rad]
        r     선수각속도 [rad/s]
        delta 실제 타각 [rad] (구동기 비활성 시 무시되고 delta_cmd 가 바로 반영)

    동역학:
        psidot   = r
        rdot     = (-r + K * delta) / T
        deltadot = sat((delta_cmd - delta) / t_delta, +-delta_rate_max)

    타각 포화 자체는 적분기(simulate_heading)가 상태를 클램프해 처리한다.
    여기서는 포화 경계 밖으로 미는 미분을 0 으로 만들어 안티와인드업만 담당한다.
    """
    r = x[1]
    delta = x[2]
    act = p.actuator

    if act.enable:
        delta_dot = (delta_cmd - delta) / act.t_delta

        # 변화율 제한
        if abs(delta_dot) > act.delta_rate_max:
            delta_dot = np.sign(delta_dot) * act.delta_rate_max

        # 포화 경계에서 바깥으로 미는 미분은 차단
        if (delta >= act.delta_max and delta_dot > 0) or (
            delta <= -act.delta_max and delta_dot < 0
        ):
            delta_dot = 0.0

        delta_eff = delta
    else:
        delta_dot = 0.0
        delta_eff = delta_cmd

    r_dot = (-r + p.K * delta_eff) / p.T
    return np.array([r, r_dot, delta_dot])
