# usv-heading-rl

무인수상정(USV)의 침로 제어 PID 게인을 강화학습으로 최적화하고, 그 결과를 검증한다.

**이 프로젝트는 AI 로 제어기를 대체하지 않는다.**
최종 제어기는 검증 가능한 PID이며, 강화학습은 게인을 찾는 최적화 도구로만 쓴다.

---

## 시작하기

```bash
pip install -e ".[dev]"

pytest                                          # 검증 (70개)
python scripts/run_step_response_check.py       # 플랜트 계단 응답 확인 (C1~C6)
python scripts/run_baseline_tuning.py           # 플랜트 특성 출력
python scripts/run_baseline_tuning.py --kp 3 --ki 0 --kd 200 --step 10
```

1단계는 Python 으로 진행한다 (`docs/decisions.md` D-002b).
MATLAB 구현은 `matlab/` 에 보류 상태로 남아 있으며, 2단계에서 재검토한다.

---

## 문서

| 문서 | 내용 |
|---|---|
| [HANDOFF.md](HANDOFF.md) | 프로젝트 인수인계. **가장 먼저 읽는다** |
| [docs/01_requirements.md](docs/01_requirements.md) | 요구사항 및 범위 정의 |
| [docs/02_plant_model.md](docs/02_plant_model.md) | 플랜트 모델과 파라미터 출처 |
| [docs/03_baseline_pid.md](docs/03_baseline_pid.md) | 베이스라인 튜닝 과정과 결과 |
| [docs/04_rl_design.md](docs/04_rl_design.md) | 보상 함수, 알고리즘, 하이퍼파라미터 |
| [docs/05_validation.md](docs/05_validation.md) | 검증 조건과 결과 |
| [docs/decisions.md](docs/decisions.md) | 설계 결정 로그 |
| [docs/journal.md](docs/journal.md) | 작업 일지 — **작업자(David) 본인 기록** |
| [docs/worklog.md](docs/worklog.md) | 작업 로그 — AI 에 위임한 작업의 진행과 막힌 지점 |

---

## 구조

```
src/usv_heading/
  plant.py         1차 Nomoto 모델 + 문헌 출처가 확인된 파라미터 세트
  sim.py           RK4 폐루프 시뮬레이션, 각도 정규화(ssa)
  control.py       PID 침로 제어기 (게인은 외부 주입)
  metrics.py       평가 지표 4종 + markdown 비교표
  plotting.py      침로/선수각속도/타각 3단 플롯
scripts/
  run_step_response_check.py   플랜트 검증 C1~C6
  run_baseline_tuning.py       수동 튜닝 도구
tests/                         pytest 70개. CI 가 매 푸시마다 실행
matlab/                        MATLAB 구현 (보류 — matlab/README.md 참조)
```

---

## 검증

문헌 계수는 **옮겨 적기만 하지 않는다.** 원 유체력 미계수에서 다시 유도해
일치하는지를 CI 가 매번 확인한다 — HANDOFF 9장("출처 없는 계수 사용 금지")에
대한 실행 가능한 대응이다.

- `test_mariner_k_t_rederived_from_hydrodynamic_derivatives` — K, T 재유도
- `test_otter_yaw_inertia_recomputed_from_mss` — M(6,6) 재계산
- `test_every_param_set_cites_a_source` — 출처 없는 세트 추가 차단

플랜트 자체는 해석해와 대조한다 (C1 상대오차 4.9e-14).

---

## 진행 단계

- [x] **1단계 준비** — 플랜트, PID, 지표, 검증, CI
- [ ] **1단계** — 베이스라인 PID 수동 튜닝 → RL 게인 최적화 → 비교표
- [ ] **2단계** — 수평면 3자유도 조종운동모델로 플랜트 교체
- [ ] **3단계** — 파랑·조류 외란 하 게인 강건성 비교

---

## 역할 경계

이 프로젝트의 목적은 "동작하는 코드"가 아니라 **"작업자가 설명할 수 있는 결과물"** 이다.
아래 항목은 AI 에 위임하지 않는다 (HANDOFF.md 5장):

- 베이스라인 PID 게인 튜닝
- 보상 함수의 항 구성과 가중치 결정
- 검증 조건 설계
- 결과의 채택 여부 판단

`docs/decisions.md` 의 `AI 활용` 항목에 어디까지 위임했는지가 건별로 기록된다.

기록도 둘로 나뉜다 — `journal.md` 는 작업자 본인의 일지이고,
`worklog.md` 는 AI 에 위임한 작업의 로그다. 두 파일의 차이가 곧
"AI 를 어디까지 썼는가"에 대한 답이다.
