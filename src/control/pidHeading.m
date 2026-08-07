function [deltaCmd, state] = pidHeading(t, psi, r, psiRef, state, h, gains, p)
% PIDHEADING  침로 유지 PID 제어기.
%
%   [deltaCmd, state] = pidHeading(t, psi, r, psiRef, state, h, gains, p)
%
%   제어칙:
%     e         = ssa(psiRef - psi)
%     deltaCmd  = Kp*e + Ki*integral(e) + Kd*(-r)
%
%   미분항에 e 의 수치미분이 아니라 -r 을 쓴다 (derivative on measurement).
%   목표 침로가 계단으로 바뀔 때 미분 킥이 생기지 않고, r 은 어차피
%   상태로 직접 측정되므로 수치미분 잡음도 없다.
%
%   ⚠ 게인 값은 이 함수가 정하지 않는다. 반드시 호출자가 gains 로 넘긴다.
%     베이스라인 게인은 작업자(David)가 수동 튜닝한다. HANDOFF.md 5장, 9장 참조.
%     이 파일에 자동 튜닝 코드를 추가하지 말 것.
%
%   입력:
%     gains   구조체. 필드 Kp, Ki, Kd (단위: rad 타각 / rad 오차 계열)
%     p       nomotoParams 구조체 (구동기 포화 한계 → 적분 안티와인드업에 사용)
%     state   내부 상태. 첫 호출 시 [] 를 넘기면 초기화된다.
%
%   사용 예 (simulateHeading 에 넘길 때):
%     ctrl = @(t,psi,r,psiRef,st,h) pidHeading(t,psi,r,psiRef,st,h,gains,p);
%     out  = simulateHeading(p, ctrl, opts);
%
%   See also simulateHeading, ssa, headingMetrics.

if isempty(state)
    state = struct('eInt', 0);
end

e = ssa(psiRef - psi);

% 미포화 지령
uUnsat = gains.Kp * e + gains.Ki * state.eInt + gains.Kd * (-r);

% 포화 한계
if p.actuator.enable && isfinite(p.actuator.deltaMax)
    uMax = p.actuator.deltaMax;
else
    uMax = Inf;
end
uSat = max(-uMax, min(uMax, uUnsat));

% 적분 안티와인드업 (conditional integration):
% 포화 중이면서 오차가 포화를 더 키우는 방향일 때만 적분을 멈춘다.
saturated = (uUnsat ~= uSat);
winding   = saturated && (sign(e) == sign(uUnsat));
if ~winding
    state.eInt = state.eInt + e * h;
end

deltaCmd = uSat;

end
