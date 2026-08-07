# usv-heading-rl

무인수상정(USV)의 침로 제어 PID 게인을 강화학습으로 최적화하고, 그 결과를 검증한다.

**이 프로젝트는 AI 로 제어기를 대체하지 않는다.**
최종 제어기는 검증 가능한 PID이며, 강화학습은 게인을 찾는 최적화 도구로만 쓴다.

---

## 시작하기

```matlab
>> startup_project              % src/ 경로 등록
>> checkEnv                     % MATLAB 버전 / 툴박스 라이선스 확인
>> run_step_response_check      % Nomoto 플랜트 검증 (C1~C6)
>> run_baseline_tuning          % 베이스라인 PID 수동 튜닝 (게인은 직접 입력)
```

1단계 코드는 **기본 MATLAB 기능만** 사용한다. Simulink·RL Toolbox 불필요.
GNU Octave 에서도 동작한다.

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
| [docs/journal.md](docs/journal.md) | 실패·시행착오 기록 |

---

## 구조

```
src/
  checkEnv.m                       환경 확인
  plant/
    nomotoParams.m                 파라미터 세트 (출처 명시)
    nomotoDerivative.m             1차 Nomoto + 구동기 상태미분
  sim/
    simulateHeading.m              폐루프 시뮬레이션 (RK4)
    ssa.m                          각도 (-pi, pi] 정규화
  control/
    pidHeading.m                   PID 침로 제어기 (게인은 외부 입력)
  analysis/
    headingMetrics.m               평가 지표 4종
    plotHeadingResponse.m          침로/선수각속도/타각 3단 플롯
    printMetrics.m                 markdown 비교표 생성
  scripts/
    run_step_response_check.m      플랜트 검증
    run_baseline_tuning.m          수동 튜닝 도구
```

---

## 진행 단계

- [x] **1단계 준비** — 저장소 구조, Nomoto 모델, PID, 지표, 검증 스크립트
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
