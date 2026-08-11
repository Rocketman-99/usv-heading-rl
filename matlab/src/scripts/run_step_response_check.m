% RUN_STEP_RESPONSE_CHECK
%
% HANDOFF.md 10장 4번: Nomoto 1차 모델 구현 및 계단 응답 확인.
%
% 타각 계단 입력에 대한 선수각속도 응답이 물리적으로 타당한지 확인한다.
% 확인 항목:
%   C1  수치적분이 해석해와 일치하는가          r(t) = K*delta*(1 - e^(-t/T))
%   C2  정상상태 선수각속도가 K*delta 인가
%   C3  63.2% 도달 시간이 T 인가
%   C4  부호가 맞는가 (양의 타각 -> 양의 선수각속도 -> 선수각 증가)
%   C5  선회 반경이 선체 길이 대비 물리적으로 타당한 범위인가
%   C6  구동기 변화율 제한이 실제로 지켜지는가
%
% 실행:
%   >> startup_project
%   >> run_step_response_check

clear; clc;

SET_NAME = 'mariner';        % nomotoParams('list') 로 다른 세트 확인 가능
DELTA_DEG = 5;               % 계단 타각 [deg]
TOL_REL = 1e-6;

p = nomotoParams(SET_NAME);
d2r = pi/180; r2d = 180/pi;

fprintf('==================================================\n');
fprintf(' Nomoto 계단 응답 확인 — 세트 "%s"\n', p.name);
fprintf('==================================================\n');
fprintf('K = %+.5f [%s]\n', p.K, p.Kunit);
fprintf('T = %+.2f s\n', p.T);
fprintf('입력 종류 : %s\n', p.inputType);
fprintf('출처 : %s\n\n', p.source);

pass = true;

%% ---- 개루프 계단 응답 (구동기 동특성 없음) --------------------------
pOL = p;
pOL.actuator.enable = false;
pOL.actuator.deltaMax = Inf;

delta = DELTA_DEG * d2r;

% tEnd = 10T. 지수 잔차 e^(-10) = 4.5e-5 이므로 C2 의 1e-3 판정이 성립한다.
% 6T 로 두면 잔차가 e^(-6) = 2.5e-3 라 C2 가 "실패"하는데, 이는 구현 오류가
% 아니라 시뮬레이션이 짧아 정상상태에 덜 도달한 것뿐이다.
opts = struct('tEnd', 10 * abs(p.T), 'h', abs(p.T)/500, 'deltaOpenLoop', delta);
ol = simulateHeading(pOL, [], opts);

% --- C1 해석해 대조 ---
rAnalytic = p.K * delta * (1 - exp(-ol.t / p.T));
errAbs = max(abs(ol.r - rAnalytic));
errRel = errAbs / max(abs(rAnalytic));
pass = report('C1 수치적분 vs 해석해', errRel < TOL_REL, ...
    sprintf('최대 상대오차 %.2e (허용 %.0e)', errRel, TOL_REL)) && pass;

% --- C2 정상상태 이득 ---
rSS      = ol.r(end);
rSSexact = p.K * delta;
pass = report('C2 정상상태 선수각속도 = K*delta', ...
    abs(rSS - rSSexact) / abs(rSSexact) < 1e-3, ...
    sprintf('r_ss = %.5f deg/s, K*delta = %.5f deg/s (잔차 e^(-tEnd/T) = %.1e)', ...
    rSS*r2d, rSSexact*r2d, exp(-opts.tEnd/abs(p.T)))) && pass;

% --- C3 시상수 ---
i63  = find(abs(ol.r) >= 0.632 * abs(rSSexact), 1, 'first');
T63  = ol.t(i63);
pass = report('C3 63.2%% 도달 시간 = T', ...
    abs(T63 - abs(p.T)) / abs(p.T) < 0.02, ...
    sprintf('T63 = %.2f s, T = %.2f s', T63, p.T)) && pass;

% --- C4 부호 ---
signOk = sign(rSS) == sign(p.K * delta) && ol.psi(end) * sign(rSS) > 0;
pass = report('C4 부호 정합 (delta>0 -> r>0 -> psi 증가)', signOk, ...
    sprintf('r_ss = %+.4f deg/s, psi(end) = %+.1f deg', rSS*r2d, ol.psi(end)*r2d)) && pass;

% --- C5 선회 반경 ---
if strcmp(p.inputType, 'rudder_angle') && isfinite(p.L) && isfinite(p.U)
    R    = p.U / abs(rSS);
    RoL  = R / p.L;
    fprintf('\n  [C5] delta = %g deg 정상선회:\n', DELTA_DEG);
    fprintf('       선회 반경 R = %.1f m = %.2f L  (L = %.2f m, U = %.3f m/s)\n', ...
        R, RoL, p.L, p.U);
    fprintf('       참고: IMO 조종성 기준은 타각 35 deg 에서 전술 직경 <= 5L.\n');
    fprintf('       선형 Nomoto 는 큰 타각에서 선회 능력을 과대평가하므로,\n');
    fprintf('       이 값은 "작은 타각에서의 선형 근사"로만 해석한다.\n');
    fprintf('       (docs/02_plant_model.md 의 선형/비선형 비교표 참조)\n');
else
    fprintf('\n  [C5] 선회 반경 확인 건너뜀 (입력이 타각이 아니거나 L, U 미정).\n');
end

%% ---- 구동기 제한 확인 ------------------------------------------------
if p.actuator.enable
    % 포화 한계를 넘는 큰 지령을 주고 변화율/포화가 지켜지는지 본다
    bigCmd = 2 * p.actuator.deltaMax;
    optsA  = struct('tEnd', 4 * abs(p.T), 'h', abs(p.T)/500, 'deltaOpenLoop', bigCmd);
    act    = simulateHeading(p, [], optsA);

    rateMax = max(abs(diff(act.delta)) / optsA.h);
    rateOk  = rateMax <= p.actuator.deltaRateMax * (1 + 1e-3);
    satOk   = max(abs(act.delta)) <= p.actuator.deltaMax * (1 + 1e-9);

    fprintf('\n');
    pass = report('C6a 타각 변화율 제한 준수', rateOk, ...
        sprintf('관측 최대 %.3f deg/s, 한계 %.3f deg/s', ...
        rateMax*r2d, p.actuator.deltaRateMax*r2d)) && pass;
    pass = report('C6b 타각 포화 준수', satOk, ...
        sprintf('관측 최대 %.2f deg, 한계 %.2f deg', ...
        max(abs(act.delta))*r2d, p.actuator.deltaMax*r2d)) && pass;
end

%% ---- 결과 ------------------------------------------------------------
fprintf('\n==================================================\n');
if pass
    fprintf(' 전체 통과 — 플랜트 구현이 해석해 및 물리적 타당성과 일치\n');
else
    fprintf(' 실패 항목 있음 — docs/journal.md 에 기록하고 원인 확인\n');
end
fprintf('==================================================\n');

%% ---- 플롯 ------------------------------------------------------------
fig = figure('Name', 'Nomoto open-loop step response');

subplot(2,1,1); hold on; grid on;
plot(ol.t, rAnalytic * r2d, 'k--', 'LineWidth', 2, 'DisplayName', '해석해');
plot(ol.t, ol.r * r2d, 'r-', 'LineWidth', 1.2, 'DisplayName', '수치적분 (RK4)');
plot([T63 T63], ylim, 'b:', 'HandleVisibility', 'off');
ylabel('r [deg/s]');
title(sprintf('%s: \\delta = %g deg 계단 입력 (K = %.4f 1/s, T = %.1f s)', ...
    p.name, DELTA_DEG, p.K, p.T));
legend('Location', 'southeast');

subplot(2,1,2); grid on;
plot(ol.t, ol.psi * r2d, 'LineWidth', 1.5);
ylabel('\psi [deg]'); xlabel('t [s]');

if exist('figures', 'dir')
    saveas(fig, fullfile('figures', sprintf('step_response_%s.png', p.name)));
    fprintf('\n그림 저장: figures/step_response_%s.png\n', p.name);
end

%% ----------------------------------------------------------------------
function ok = report(name, ok, detail)
if ok, mark = 'PASS'; else, mark = 'FAIL'; end
fprintf('  [%s] %-40s %s\n', mark, name, detail);
end
