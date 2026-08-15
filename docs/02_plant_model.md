# 02. 플랜트 모델과 파라미터 출처

HANDOFF.md 10장 4번에 해당한다.
1단계 플랜트는 **1차 Nomoto 모델**이다.

---

## 1. 모델 정의

```
T · ṙ + r = K · δ
ψ̇ = r
```

| 기호 | 의미 | 단위 |
|---|---|---|
| ψ | 선수각 (heading) | rad |
| r | 선수각속도 (yaw rate) | rad/s |
| δ | 타각 (rudder angle) | rad |
| K | Nomoto 이득 | 1/s |
| T | Nomoto 시상수 | s |

전달함수:

```
ψ(s)      K
────  =  ───────────
δ(s)     s (1 + T s)
```

계단 타각 δ₀ 에 대한 해석해 (r(0) = 0):

```
r(t) = K · δ₀ · (1 − e^(−t/T))
```

구현: `src/usv_heading/plant.py`, `src/usv_heading/sim.py`
검증: `tests/test_plant.py`, `tests/test_sim.py`
실행: `python scripts/run_step_response_check.py`

---

## 2. 파라미터 세트

`src/usv_heading/plant.py` 에 세 세트가 등록되어 있다.
**어떤 세트를 이 프로젝트의 플랜트로 채택할지는 아직 결정되지 않았다** — `decisions.md` D-004 (OPEN).

### 2.1 `mariner` — Mariner 급 화물선 (현재 기본값)

| 항목 | 값 |
|---|---|
| K | 0.185 1/s |
| T | 107.3 s (= T₁ + T₂ − T₃) |
| T₁, T₂, T₃ | 118 s, 7.8 s, 18.5 s |
| L (수선간장) | 160.93 m |
| U₀ (기준 속력) | 7.7175 m/s (15 knots) |
| 타각 한계 | ±40 deg |
| 타각 변화율 한계 | 5 deg/s |

**출처**

- K, T₁, T₂, T₃ — Fossen, *MSS (Marine Systems Simulator)*, `mssExamples/exNomoto.m`,
  <https://github.com/cybergalactic/MSS> (2026-08-07 확인)
- 원 출처 — M. S. Chislett and J. Strøm-Tejsen (1965),
  *"Planar Motion Mechanism Tests and Full-Scale Steering and Manoeuvring Predictions
  for a Mariner Class Vessel"*, **International Shipbuilding Progress 12(129), 201–224**,
  <https://doi.org/10.3233/ISP-1965-1212902>
  (동 내용의 기관 보고서는 Report **Hy-6**, Hydro- and Aerodynamics Laboratory, Lyngby, Denmark)

  > ⚠ **인용 오류 정정 (2026-08-15)**: MSS `mariner.m` 헤더는 이 논문을 "Technical Report Hy-5"
  > 라고 적고 있고, 본 저장소도 그대로 옮겨 적었었다. 확인 결과 **Hy-5 는 Abkowitz (1964),
  > *Lectures on Ship Hydrodynamics — Steering and Manoeuvrability*** 의 번호다
  > ([TRID Hy-5](https://trid.trb.org/View/159100)).
  > 보고서 번호는 출처마다 엇갈리므로 **저널판(ISP 12(129))으로 인용하는 것이 안전하다.**
- L, U₀, 구동기 한계 — MSS `CRAFT/SHIP/models/mariner.m`
  (타각 한계 40 deg 는 IMO 의 35 deg rudder execute 규정을 만족시키기 위한 값이라고 동 파일에 명시)

**검증 (2026-08-07 수행)**

문헌 값을 그대로 옮겨 쓰지 않고, 같은 저장소의 **선형 유체력 미계수로부터 직접 재유도**해 일치를 확인했다.

`mariner.m` 의 선형 미계수 (무차원, prime system):

```
m  = 798e-5,  Iz = 39.2e-5,  x_G = −0.023
Y_v̇ = −748e-5    N_v̇ =  4.646e-5
Y_ṙ = −9.354e-5  N_ṙ = −43.8e-5
Y_v = −1160e-5   N_v = −264e-5
Y_r = −499e-5    N_r = −166e-5
Y_δ =  278e-5    N_δ = −139e-5
```

**⚠ Y_r, N_r 은 강체 구심항을 이미 포함한 "총(total)" 미계수다 (2026-08-15 확인)**

선형 조종방정식을 교과서식으로 쓰면 sway 식에 강체 구심항 `m·U·r` 이 나타나고,
그 결과 미계수가 `Y_r − m·U`, `N_r − m·x_G·U` 로 바뀐다. **이 계수 세트에는 그렇게 하면 안 된다.**

- MSS `mariner.m` 의 운동방정식에 `m*u*r` 형태의 항이 존재하지 않는다.
  `Y = Yv*v + Yr*r + ...` 로 Y_r 이 그대로 쓰인다
  (`Yru*r*u` 는 u 가 기준속력 대비 섭동이라 정격속도에서 0 이 되는 속도보정항이며 별개다).
- 구심항을 명시적으로 더하면 특성방정식 상수항이 `+6.08e-6 → −1.71e-5` 로 부호가 뒤집혀
  **침로 불안정**(극점 +0.417)이 되고, 문헌값 T₁ = 118 s, T₂ = 7.8 s (둘 다 양수)와 모순된다.

즉 아래 재유도는 구심항을 따로 더하지 않는 것이 옳다.

선형 sway–yaw 방정식에서 r(s)/δ(s) 를 구한 결과:

| | 재유도 결과 | `exNomoto.m` 기재값 |
|---|---|---|
| K | 0.18499 1/s | 0.185 |
| T₁ | 117.98 s | 118 |
| T₂ | 7.76 s | 7.8 |
| T₃ | 18.53 s | 18.5 |
| **T = T₁+T₂−T₃** | **107.21 s** | **107.3** |

일치한다. K, T 는 무차원 값이 아니라 **차원을 가진 값**이며, 무차원화 시간 척도는 L/U = 20.85 s 다.

**⚠ 선형 근사의 유효 범위 (중요)**

같은 저장소의 **비선형** Mariner 모델(`mariner.m` 전체)을 적분해, 타각별 실효 이득을
`K_eff = (r_ss(+δ) − r_ss(−δ)) / 2δ` 로 구하고 선형값 0.185 와 비교했다.
(비선형 모델에는 Y₀, N₀ 비대칭 항이 있어 δ = 0 에서도 r ≈ 0.17 deg/s 의 편향이 남는다.
중앙차분으로 이 편향을 제거한 뒤 비교했으며, 비교를 위해 편향 항을 0 으로 둔 변형도 함께 계산했다.)

| 타각 δ [deg] | K_eff [1/s] | K_eff / 0.185 | 편향항 제거 시 비 |
|---|---|---|---|
| 0.1 | 0.1126 | 0.61 | **1.00** |
| 0.5 | 0.1146 | 0.62 | 0.95 |
| 1 | 0.1192 | 0.64 | 0.85 |
| 2 | 0.1147 | 0.62 | 0.67 |
| 5 | 0.0769 | 0.42 | 0.42 |
| 10 | 0.0493 | 0.27 | 0.27 |
| 20 | 0.0291 | 0.16 | 0.16 |
| 35 | 0.0174 | 0.09 | 0.09 |

읽는 법:

1. δ → 0 에서 비가 정확히 1.00 이 된다. **선형 Nomoto 파라미터가 원 모델의 정확한 선형화임이 확인된다.**
2. 그러나 실선 운항 영역(δ = 5~35 deg)에서 선형 모델은 선회 능력을 **2~10배 과대평가**한다.
   비선형 감쇠 항(N_vvr, N_vvv 등)이 지배적이 되기 때문이다.
3. 따라서 **1단계에서 다루는 침로 변경/타각의 크기는 작게 잡아야 한다.**
   대략 |δ| ≲ 1 deg 에서 이득 오차 15% 이내, |δ| ≲ 2 deg 에서 33% 수준이다.

이 한계는 숨기지 않고 결과물에 명시한다. 면접 질문 "RL이 낸 게인을 실제 해상에서
신뢰할 수 있는가?" 에 대한 답의 일부다 — **1단계 결과는 선형 영역에서만 유효하며,
그래서 2단계에서 3자유도 모델로 교체한다.**

재현 방법: `python scripts/run_step_response_check.py` (선형 검증 C1~C6).
비선형 대조는 MSS 저장소를 받아 `mssExamples/exKT.m` 를 참고한다.

---

### 2.2 `tanker` — 유조선 (침로 불안정)

| 항목 | 값 |
|---|---|
| K | −0.019 1/s |
| T | −153.7 s (= −124.1 + 16.4 − 46.0) |

**출처**: Fossen, MSS, `mssExamples/exNomoto.m` ("Oil tanker") (2026-08-07 확인)

T < 0 이므로 개루프 침로 불안정선이다. 1단계 대상이 아니다.
3단계 이후 "게인이 플랜트 특성 변화에 얼마나 민감한가"를 보이는 대조군 후보.

⚠ L, U, 구동기 한계는 MSS 에 명시되어 있지 않다. 사용하려면 출처를 보완해야 한다.

---

### 2.3 `otter` — Otter USV (Maritime Robotics)

| 항목 | 값 |
|---|---|
| 입력 | **요 모멘트 τ_N [N·m]** (타각 아님) |
| K | 1 / M(6,6) = 0.024695 (rad/s)/(N·m) |
| T | 1.0 s |
| M(6,6) | 40.4938 kg·m² |
| L, B, m | 2.0 m, 1.08 m, 55 kg (+ 탑재 25 kg) |
| 최대 속력 | 6 knots = 3.086 m/s |

**출처**

- T = 1 s 및 K = T / M(6,6) — MSS `CRAFT/USV/SIMotter.m` 97–99행
- M(6,6) — MSS `CRAFT/USV/models/otter.m` 의 M = M_RB + M_A 구성을
  mp = 25 kg, rp = [0 0 0] 조건으로 재계산해 확인 (2026-08-07)
- 제원 — 동 파일

**⚠ 이 세트를 쓰려면 먼저 정리해야 할 것 두 가지**

1. **입력이 타각이 아니라 요 모멘트다.** Otter 는 차동추진(differential thrust) USV 라
   물리적 러더가 없다. HANDOFF 7장의 "러더 사용량 = 타각 변화율의 적분" 지표를
   모멘트(또는 좌우 프로펠러 회전수 차) 기준으로 다시 정의해야 한다.
2. **T = 1 s 는 동정(identification)값이 아니다.** MSS 는 오토파일럿 설계 편의를 위해
   T = 1 s 로 두고 K 를 M(6,6) 에서 역산한다. 즉 실험으로 얻은 문헌 계수가 아니다.
   "문헌의 표준 선형 값" 이라는 HANDOFF 1단계 요구를 엄밀히 만족하지 않는다.

---

## 3. 구동기(타기) 모델

순수 Nomoto 에는 구동기가 없지만, `러더 사용량` 지표와 RL 의 해 탐색이
물리적으로 의미를 가지려면 구동기 한계가 필요하다. 없으면 학습이 무한대 타각률을
쓰는 해를 찾아낼 수 있다.

구현 (`plant.py::nomoto_derivative`):

```
δ̇ = sat( (δ_cmd − δ) / T_δ , ±δ̇_max )
δ  = sat( δ , ±δ_max )
```

| 파라미터 | mariner 기본값 | 출처 |
|---|---|---|
| δ_max | 40 deg | MSS `mariner.m` (IMO 35 deg rudder execute 대응) |
| δ̇_max | 5 deg/s | MSS `mariner.m` |
| T_δ | 1.0 s | MSS `mariner.m` 은 T_δ = 1 s 에 해당하는 형태(`delta_dot = delta_c - delta`)를 쓴다 |

`p.with_actuator(enable=False)` 로 끄면 순수 Nomoto 가 된다.
계단 응답 해석해 검증(C1)은 이 상태에서 수행한다.

⚠ 구동기 모델을 포함할지 여부는 `decisions.md` D-006 (OPEN) — 작업자 확인 필요.

---

## 4. 확인된 사항 / 미확인 사항

**확인됨**

아래는 `scripts/run_step_response_check.py` 실행 결과다
(2026-08-07, δ = 5 deg, tEnd = 10T, h = T/500).
같은 항목이 `tests/test_sim.py` 에 pytest 로도 들어 있어 CI 가 매 푸시마다 확인한다.

| | 항목 | 결과 |
|---|---|---|
| C1 | 수치적분(RK4) vs 해석해 | 상대오차 **4.9e-14** (허용 1e-6) ✔ |
| C2 | 정상상태 r = K·δ | 0.92496 vs 0.92500 deg/s, 상대오차 4.5e-5 ✔ |
| C3 | 63.2% 도달 시간 = T | **107.30 s** vs T = 107.3 s ✔ |
| C4 | 부호 정합 | δ>0 → r = +0.925 deg/s → ψ 증가 ✔ |
| C5 | 선회 반경 | 478 m = **2.97 L** (δ = 5 deg) — 아래 주석 참조 |
| C6a | 타각 변화율 제한 | 관측 5.000 ≤ 한계 5.0 deg/s ✔ |
| C6b | 타각 포화 | 관측 40.00 ≤ 한계 40.0 deg ✔ |

> **C5 해석**: 선형 모델은 δ = 5 deg 에서 선회 반경 2.97 L 을 낸다.
> 같은 조건에서 **비선형 모델은 6.50 L** 이다 (앞 절의 r_ss = 0.4228 deg/s).
> 2.2배 차이는 위 유효범위 표의 K_eff/K = 0.42 와 정확히 대응한다.
> 즉 C5 는 "구현이 틀렸다"가 아니라 "선형 근사의 한계가 예상대로 나타난다"는 확인이다.

그 외:

- [x] K, T 가 Chislett & Strøm-Tejsen 선형 미계수의 정확한 선형화임을 재유도로 확인
      → `tests/test_plant.py::test_mariner_k_t_rederived_from_hydrodynamic_derivatives`
      가 CI 에서 매번 재계산해 대조한다. 문헌값을 옮겨 적기만 하면 오타를 못 잡으므로,
      원 미계수에서 다시 유도해 일치를 확인하는 방식으로 HANDOFF 9장을 실행 가능하게 만들었다
- [x] Otter M(6,6) = 40.49375 kg·m² 재계산 확인 → `::test_otter_yaw_inertia_recomputed_from_mss`
- [x] 모든 파라미터 세트가 출처를 갖는지 CI 가 강제 → `::test_every_param_set_cites_a_source`
- [x] 선수각 해석해 ψ(t) = K·δ·(t − T(1−e^(−t/T))) 와 일치 (상대오차 5.5e-15)
- [x] RK4 4차 수렴 차수 확인
- [x] 선형 근사의 타각별 유효 범위 정량화

**미확인 / 열린 항목**

- [ ] 대상 선박 확정 (D-004)
- [ ] 구동기 모델 포함 여부 확정 (D-006)
- [ ] `tanker` 세트의 L, U, 구동기 한계 출처 보완
