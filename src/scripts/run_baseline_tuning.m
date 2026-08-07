% RUN_BASELINE_TUNING
%
% HANDOFF.md 10장 5번: 베이스라인 PID 수동 튜닝 도구.
%
% ┌────────────────────────────────────────────────────────────────────┐
% │ 이 스크립트는 게인을 "찾아주지 않는다".                              │
% │ 작업자(David)가 아래 GAINS 값을 직접 바꿔가며 응답과 지표를 본다.    │
% │ HANDOFF.md 5장: 베이스라인 PID 게인 튜닝은 대행 금지 항목이다.       │
% │ HANDOFF.md 9장: 자동 튜닝 스크립트로 대체하는 것은 금지다.           │
% │ 이 파일에 pidtune / fminsearch / 격자탐색 등을 추가하지 말 것.       │
% └────────────────────────────────────────────────────────────────────┘
%
% 사용법:
%   1) 아래 GAINS.Kp, Ki, Kd 에 값을 넣는다
%   2) >> run_baseline_tuning
%   3) 플롯(침로/선수각속도/타각)과 지표표를 본다
%   4) 만족스러울 때까지 1~3 반복
%   5) 채택한 게인과 그렇게 정한 이유를 docs/03_baseline_pid.md 에 기록
%   6) 튜닝 과정에서 버린 시도도 docs/journal.md 에 남긴다
%
% 튜닝 시 볼 것 (HANDOFF.md 8장 3항):
%   - 지표만 보지 말 것. 타각(3번째 subplot)이 지속 진동하면 그 게인은 버린다.
%   - 정상상태 오차가 남으면 Ki, 오버슈트가 크면 Kd 를 먼저 본다.

clear; clc;

%% ---- 1. 튜닝 대상 -----------------------------------------------------
SET_NAME  = 'mariner';
p = nomotoParams(SET_NAME);

d2r = pi/180; r2d = 180/pi;

%% ---- 2. 게인 (작업자가 직접 입력) -------------------------------------
% 단위: deltaCmd[rad] = Kp*e[rad] + Ki*int e[rad*s] + Kd*(-r)[rad/s]
%
% NaN 이면 스크립트가 멈춘다. 값을 넣기 전에는 실행되지 않는다.
GAINS.Kp = NaN;   % [-]
GAINS.Ki = NaN;   % [1/s]
GAINS.Kd = NaN;   % [s]

if any(isnan([GAINS.Kp, GAINS.Ki, GAINS.Kd]))
    fprintf('\n');
    fprintf('  게인이 아직 비어 있다.\n');
    fprintf('  %s 의 GAINS 를 채운 뒤 다시 실행할 것.\n', mfilename('fullpath'));
    fprintf('\n');
    fprintf('  참고 — 이 플랜트의 개루프 특성:\n');
    fprintf('    K = %+.5f 1/s,  T = %+.2f s\n', p.K, p.T);
    fprintf('    전달함수  psi(s)/delta(s) = K / (s*(1 + T*s))\n');
    fprintf('    타각 한계 %.1f deg, 변화율 한계 %.1f deg/s\n', ...
        p.actuator.deltaMax*r2d, p.actuator.deltaRateMax*r2d);
    fprintf('\n');
    fprintf('  게인 값을 무엇으로 시작할지는 작업자가 정한다.\n');
    fprintf('  (HANDOFF.md 5장 — 대행 금지 항목)\n\n');
    return
end

%% ---- 3. 시험 조건 -----------------------------------------------------
% 아래 조건은 "튜닝용" 기본 조건이다.
% 최종 검증 조건 설계는 작업자 담당이다 (HANDOFF.md 5장).
% 확정하면 docs/05_validation.md 에 기록한다.
STEP_DEG = 10;                 % 목표 침로 계단 [deg]
                               % 주의: 선형 Nomoto 의 유효 범위는 작은 타각이다.
                               % docs/02_plant_model.md 의 선형/비선형 비교 참조.

opts = struct( ...
    'tEnd',   6 * abs(p.T), ...
    'h',      abs(p.T) / 500, ...
    'psi0',   0, ...
    'r0',     0, ...
    'delta0', 0, ...
    'psiRef', STEP_DEG * d2r);

%% ---- 4. 시뮬레이션 ----------------------------------------------------
ctrl = @(t, psi, r, psiRef, st, h) pidHeading(t, psi, r, psiRef, st, h, GAINS, p);
out  = simulateHeading(p, ctrl, opts);

%% ---- 5. 지표 및 플롯 --------------------------------------------------
m = headingMetrics(out, struct('band', 0.05, 'ssFrac', 0.30));

fprintf('\n게인: Kp = %.4f, Ki = %.5f, Kd = %.4f   |   계단 %g deg, 플랜트 "%s"\n', ...
    GAINS.Kp, GAINS.Ki, GAINS.Kd, STEP_DEG, p.name);
printMetrics({m}, {sprintf('Kp=%.3g Ki=%.3g Kd=%.3g', GAINS.Kp, GAINS.Ki, GAINS.Kd)});

plotHeadingResponse(out, {'baseline PID'}, ...
    sprintf('%s: 계단 %g deg, Kp=%.3g Ki=%.3g Kd=%.3g', ...
    p.name, STEP_DEG, GAINS.Kp, GAINS.Ki, GAINS.Kd));

%% ---- 6. 다음 단계 안내 ------------------------------------------------
fprintf('타각 이력(3번째 subplot)에 지속 진동이 보이면 지표가 좋아도 채택하지 않는다.\n');
fprintf('채택 시 기록할 것:\n');
fprintf('  docs/03_baseline_pid.md  게인 값, 시험 조건, 위 지표표, 응답 그림\n');
fprintf('  docs/journal.md          버린 시도와 그 이유\n');
fprintf('  docs/decisions.md        D-005 (베이스라인 게인 채택 근거)\n\n');
