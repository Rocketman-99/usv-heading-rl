# MATLAB 구현 (보류)

2026-08-07 에 작성한 MATLAB 구현이다. **현재 사용하지 않는다.**

## 왜 보류인가

`docs/decisions.md` D-002b 에서 1단계를 Python 으로 진행하기로 결정했다.
가장 큰 이유는 **검증 반복 비용**이다. 개발 환경(컨테이너)에 MATLAB 이 없어
코드를 작성만 하고 실행하지 못했고, 실제로 이 폴더의 코드는 **한 번도 실행된 적이 없다.**

Python 으로 옮긴 뒤 같은 로직에 대해 pytest 70개를 작성했더니
**테스트 4개가 실패했다.** 그중 하나는 실제 이해 오류였다 —
"적분기가 계단 목표의 정상상태 오차를 없앤다"고 가정했는데, 이 플랜트는
`K/(s(1+Ts))` 로 이미 적분기를 갖는 type 1 이라 P 제어만으로 오차가 0 이었다.
MATLAB 으로 계속했다면 베이스라인 튜닝 중에 Ki 를 잘못 키웠을 것이다.

## 이 코드의 상태

- ⚠ **미실행·미검증.** 문법 오류가 남아 있을 수 있다
- 알고리즘 자체는 Python 판과 동일하며, Python 판은 검증되었다
- 위에 적은 오류 4건은 여기 반영되어 있지 않다

## 언제 다시 쓰나

2단계(3자유도 모델)에서 Simulink 를 쓰기로 하면 이 코드가 출발점이 된다.
그때 다음을 함께 한다.

1. 위 4건 수정 반영
2. `matlab-actions/setup-matlab` 으로 CI 추가
   (public 저장소 + GitHub-hosted runner 는 라이선스 토큰 불필요)
3. Python 판과 동일 조건에서 결과 일치 확인 — 교차 검증 자체가 검증 항목이 된다

## 파일

```
startup_project.m              경로 등록
src/checkEnv.m                 환경 확인 (이건 실행됨. 결과는 docs/01_requirements.md)
src/plant/nomotoParams.m       파라미터 세트
src/plant/nomotoDerivative.m   상태미분
src/sim/simulateHeading.m      RK4 폐루프 시뮬레이션
src/sim/ssa.m                  각도 정규화
src/control/pidHeading.m       PID
src/analysis/headingMetrics.m  지표
src/analysis/plotHeadingResponse.m
src/analysis/printMetrics.m
src/scripts/run_step_response_check.m
src/scripts/run_baseline_tuning.m
```
