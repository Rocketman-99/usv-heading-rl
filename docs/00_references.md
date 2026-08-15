# 00. 참고자료 색인

이 프로젝트가 실제로 인용하거나 학습에 쓰는 자료만 모은다.
링크는 **2026-08-15 접속 확인**했다.

---

## 1. 1차 문헌 (인용용)

코드/문서에서 계수나 유도를 인용할 때 쓰는 원출처다.

| 약칭 | 서지 | 이 프로젝트에서 쓰이는 곳 |
|---|---|---|
| **Chislett & Strøm-Tejsen (1965)** | M. S. Chislett, J. Strøm-Tejsen, *"Planar Motion Mechanism Tests and Full-Scale Steering and Manoeuvring Predictions for a Mariner Class Vessel"*, **International Shipbuilding Progress 12(129), 201–224**. <https://doi.org/10.3233/ISP-1965-1212902> | `mariner` 파라미터 세트의 선형 유체력 미계수 원출처 (`plant.py`, `02_plant_model.md`) |
| **Nomoto et al. (1957)** | K. Nomoto, T. Taguchi, K. Honda, S. Hirano, *"On the steering qualities of ships"*, *International Shipbuilding Progress* 4(35), 354–370 | 2차 Nomoto 모델 정의와 1차 감차 T = T₁ + T₂ − T₃ 의 최초 제시 |
| **Davidson & Schiff (1946)** | K. S. M. Davidson, L. I. Schiff, *"Turning and course keeping qualities"*, *SNAME Transactions* | 선형 sway–yaw 조종방정식(Davidson–Schiff 모델) 자체 |
| **Abkowitz (1964)** | M. A. Abkowitz, *Lectures on Ship Hydrodynamics — Steering and Manoeuvrability*, **Report Hy-5**, Hydro- and Aerodynamics Laboratory, Lyngby | 유체력의 Taylor 전개로 ∂Y/∂v 같은 미계수를 정의하는 근거 |
| **SNAME (1950)** | *Nomenclature for Treating the Motion of a Submerged Body Through a Fluid*, Technical & Research Bulletin 1-5 | 부호규약, 무차원화(prime system) |

> ### ⚠ 보고서 번호 혼동 주의
>
> MSS `mariner.m` 헤더는 Chislett & Strøm-Tejsen (1965) 을 **"Technical Report Hy-5"** 로 적고 있고,
> 이를 인용한 2차 문헌이 다수 있다. 그러나 **Hy-5 는 Abkowitz (1964) 의 번호**다
> ([TRID Hy-5 레코드](https://trid.trb.org/View/159100)).
> Chislett 의 기관 보고서는 Hy-6 로 알려져 있으나 출처마다 엇갈린다.
> **→ 저널판(ISP 12(129))으로 인용하는 것이 안전하다.** 자세한 경위는 `02_plant_model.md` §2.1.

---

## 2. Fossen 교재 — *Handbook of Marine Craft Hydrodynamics and Motion Control*

- Thor I. Fossen, **2nd Edition**, John Wiley & Sons, April 2021, ISBN 978-1-119-57505-4
- NTNU 대학원 과목 **TTK4190** 교재
- 부속자료 저장소: <https://github.com/cybergalactic/FossenHandbook>

**⚠ 저장소에 교재 본문 PDF 는 없다.** 상업 출판물이라 README + 이미지뿐이고,
아래 **강의 슬라이드**와 정오표 링크만 제공한다. 슬라이드만으로도 유도 과정은 대부분 따라갈 수 있다.

- **정오표**: [Errata.pdf](https://www.dropbox.com/scl/fi/s6icfdbt9hoqkke4ea3vt/Errata.pdf?rlkey=tu10g93tkcmgpwqtmxgr8s580&st=qpstuvu0&dl=0)

### 강의 슬라이드

우선순위 표시: ★★★ 지금 필요 / ★★ 2단계에서 필요 / ★ 참고

| | 장 | 슬라이드 | 우선순위 | 비고 |
|:-:|---|:-:|:-:|---|
| | **Part 1: Marine Craft Hydrodynamics** | | | |
| 1 | Introduction to Part I | [Ch1](https://www.dropbox.com/scl/fi/0t35ol7qjkbtq2y2tjtmr/Ch1.pdf?rlkey=733g820vq4vfxdn8pd9zl8xt9&dl=0) | ★ | |
| 2 | Kinematics | [Ch2](https://www.dropbox.com/scl/fi/i7vlggjiipr8vpp6j1evo/Ch2.pdf?rlkey=w6o0au5nphl3h7y449tziior9&dl=0) | ★★ | 좌표계, ψ̇ = r 의 근거 |
| 3 | Rigid-Body Kinetics | [Ch3](https://www.dropbox.com/scl/fi/szawrpaykezjodki8t3jq/Ch3.pdf?rlkey=54g895sql59x4gssoa7nextir&dl=0) | ★★★ | **구심항 m·U·r 이 어디서 나오는지** |
| 4 | Hydrostatics | [Ch4](https://www.dropbox.com/scl/fi/td0dwv4cgjxmvdg8dhtg9/Ch4.pdf?rlkey=fcbxt5sa0kh826jk783tb7mqt&dl=0) | ★ | |
| 5 | Seakeeping Models | [Ch5](https://www.dropbox.com/scl/fi/7yuix37gbad59qma3hi1x/Ch5.pdf?rlkey=zi16uwq78d418v03lh1mdtfb9&dl=0) | ★ | 파랑 중 운동 (3단계 이후) |
| 6 | **Maneuvering Models** | [Ch6](https://www.dropbox.com/scl/fi/f2ipl6gxs9a826b6i06dw/Ch6.pdf?rlkey=utqhkxv7y353rittx1dnte0fo&dl=0) | **★★★** | **부가질량, 유체력 미계수, M·ẋ + N·x = b·δ. Step 5 유도의 본체** |
| 7 | **Autopilot Models for Course and Heading Control** | [Ch7](https://www.dropbox.com/scl/fi/rth0nv12xsrf0ar8bsbwv/Ch7.pdf?rlkey=bjpndrhdfuxzc8281fx3n2hk8&dl=0) | **★★★** | **1차/2차 Nomoto, T = T₁+T₂−T₃ 감차. 우리 플랜트 그 자체** |
| 8 | Models for Underwater Vehicles | [Ch8](https://drive.google.com/file/d/1NEemmPazyFELrvziXi31iQZea1xYBPg9/view?usp=share_link) | ★ | |
| 9 | Control Forces and Moments | [Ch9](https://drive.google.com/file/d/11A74KPYcGeAzU2rVSkWgHbi9CvvBesRa/view?usp=share_link) | ★★ | **차동추진 제어배분 — otter 채택 시 필요** |
| 10 | Environmental Forces and Moments | [Ch10](https://drive.google.com/open?id=1-Dk0AE7UrPKBOPdeKazJSIPymbp8x6cR&usp=drive_fs) | ★★ | 외란(바람/파/조류) 주입 시 |
| | **Part 2: Motion Control Systems** | | | |
| 11 | Introduction to Part II | [Ch11](https://www.dropbox.com/scl/fi/sryr1340ilvjv9dpfs62t/Ch11.pdf?rlkey=grlws4qw8edl69s38dqivqfkp&dl=0) | ★ | |
| 12 | Guidance Systems | [Ch12](https://www.dropbox.com/scl/fi/mf5ms14t9pp7mfe4qgf6w/Ch12.pdf?rlkey=odw8avz86a02w8tg6wnfmvu79&dl=0) | ★★ | 기준모델(reference model), LOS 유도 |
| 13 | Model-Based Navigation Systems | [Ch13](https://drive.google.com/file/d/1K1RxgbNbvrErOX3CBtbcIcriACYm9Sm6/view?usp=share_link) | ★ | |
| 14 | Inertial Navigation Systems | [Ch14](https://www.dropbox.com/scl/fi/68ky3k4is0gahpxi3h010/Ch14.pdf?rlkey=6yq5djch0ycna9h69qz6xh4lz&dl=0) | ★ | |
| 15 | **Motion Control Systems** | [Ch15](https://drive.google.com/file/d/1bJlyDYg5Le1PTa_0voSzYAO00ayzu9TA/view?usp=share_link) | **★★★** | **PID 침로제어, 극점배치 게인 설계 — D-005 기준선** |
| 16 | Advanced Motion Control Systems | [Ch16](https://drive.google.com/file/d/1QepdGD2JdRnh8tlzRaXnrugr78LAoxQz/view?usp=share_link) | ★★ | 백스테핑, 슬라이딩모드 등 (비교군 후보) |
| | **Part III: Appendices** (슬라이드 없음) | | | |
| A | Nonlinear Stability Theory | — | | |
| B | Numerical Methods | — | | RK4 |
| C | Model Transformations | — | | |
| D | **Non-dimensional Equations of Motion** | — | ★★★ | prime system. `test_plant.py` 재유도의 근거 |

**최소 학습 경로**: Ch3 → Ch6 → Ch7 → Ch15. 이 4개면 1단계에 필요한 이론은 다 커버된다.

---

## 3. 코드 저장소

Fossen 계정(`cybergalactic`)의 공개 저장소는 아래 셋이 전부다.

### 3.1 MSS (Marine Systems Simulator) — MATLAB/Octave

<https://github.com/cybergalactic/MSS> · MIT · 활발히 유지보수됨

**현재 이 프로젝트가 파라미터를 가져다 쓰는 저장소.** MATLAB 없이 **GNU Octave 로 무료 실행 가능**하다.

| 디렉토리 | 내용 | 우리가 쓴 파일 |
|---|---|---|
| `CRAFT` | 선박/수중체 모델 + 시뮬레이션 스크립트 | `SHIP/models/mariner.m`, `USV/models/otter.m`, `USV/SIMotter.m` |
| `mssExamples` | 교재 예제 | **`exNomoto.m`** — K = 0.185, T₁/T₂/T₃ = 118/7.8/18.5 출처 |
| `GNC` | 유도·항법·제어 함수 | |
| `HYDRO` | WAMIT/ShipX 유체동역학 데이터 처리 | |
| `INS` | 칼만필터 기반 관성항법 | |
| `FDI` | 방사력 모델 동정 툴박스 | |
| `SIMULINK` | Simulink 블록/템플릿 | |
| `mssDemos` | 데모 | |

### 3.2 PythonVehicleSimulator — Python

<https://github.com/cybergalactic/PythonVehicleSimulator> · MIT · 최종 갱신 2025-05

MSS 의 Python 이식판. **이 프로젝트가 Python 이라 2단계 3자유도 모델의 1순위 참조 대상이다.**
`pip install -e .` 로 설치, `python3 main.py` 실행.

수록 모델 10종: **Otter (2 m USV)**, Frigate (100 m), Tanker (304.8 m, 천수효과),
Supply vessel (76.2 m), ROV Zefakkel (54 m), Semisubmersible (84.5 m),
Ship Clarke83 (선형 조종모델), DSRV, REMUS100, Torpedo.

**`vehicles/otter.py` 확인 결과 (2026-08-15) — D-004 에 직접 관련:**

```
u_control = [n1, n2]    # n1: 좌현 프로펠러 축회전수 [rad/s]
                        # n2: 우현 프로펠러 축회전수
```

- **6자유도** (surge/sway/heave/roll/pitch/yaw), 제어배분(control allocation) 포함
- 제어모드: `"stepInput"`, `"headingAutopilot"`
- 침로 오토파일럿은 **극점배치** (`wn = 2.5`, `zeta = 1`), 기준모델 `wn_d = 0.5`, `r_max = 10 deg/s`

즉 `plant.py` 의 `otter` 세트(요 모멘트 입력, T = 1 s 가정값)보다 **출처 신뢰도와 물리 충실도가 훨씬 높다.**
otter 를 채택한다면 이쪽을 참조모델로 삼아야 한다 — `decisions.md` D-004 참조.

### 3.3 FossenHandbook — 교재 부속자료

<https://github.com/cybergalactic/FossenHandbook> · 위 2장 참조. 코드는 없다.

---

## 4. 사용 원칙

`HANDOFF.md` 9장: **문헌 계수를 출처 없이 사용하지 않는다.**

- 새 계수를 넣을 때는 `NomotoParams.source` 에 출처를 적는다
  (`tests/test_plant.py::test_every_param_set_cites_a_source` 가 CI 에서 강제)
- 가능하면 **원 미계수에서 재유도해 대조**한다
  (`::test_mariner_k_t_rederived_from_hydrodynamic_derivatives`)
- 2차 문헌(MSS 헤더 포함)의 서지정보는 **그대로 믿지 않는다.**
  위 Hy-5/Hy-6 건이 실제 사례다
