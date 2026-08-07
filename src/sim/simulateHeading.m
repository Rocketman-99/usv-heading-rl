function out = simulateHeading(p, ctrl, opts)
% SIMULATEHEADING  Nomoto 침로 제어 폐루프 시뮬레이션 (고정 스텝 RK4).
%
%   out = simulateHeading(p, ctrl, opts)
%
%   입력:
%     p     nomotoParams 구조체
%     ctrl  제어기. 두 가지 형태를 받는다.
%             (a) 함수 핸들  [deltaCmd, ctrlState] = ctrl(t, psi, r, psiRef, ctrlState, h)
%             (b) []         개루프. opts.deltaOpenLoop 를 지령으로 사용
%     opts  옵션 구조체 (모든 필드 선택):
%             .tEnd            시뮬레이션 종료 시각 [s]        기본 p.T * 8
%             .h               적분 스텝 [s]                   기본 p.T / 200
%             .psi0            초기 선수각 [rad]               기본 0
%             .r0              초기 선수각속도 [rad/s]         기본 0
%             .delta0          초기 타각 [rad]                 기본 0
%             .psiRef          목표 침로. 스칼라 [rad] 또는 함수 핸들 @(t)  기본 0
%             .ctrlState       제어기 초기 상태                기본 []
%             .deltaOpenLoop   개루프 지령. 스칼라 또는 @(t)   기본 0
%
%   출력 out:
%     .t          [N x 1] 시각 [s]
%     .psi        [N x 1] 선수각 [rad]
%     .r          [N x 1] 선수각속도 [rad/s]
%     .delta      [N x 1] 실제 타각 [rad]
%     .deltaCmd   [N x 1] 지령 타각 [rad]
%     .psiRef     [N x 1] 목표 침로 [rad]
%     .err        [N x 1] 침로 오차 ssa(psiRef - psi) [rad]
%     .p, .opts   입력 그대로 (재현성 기록용)
%
%   See also nomotoParams, nomotoDerivative, headingMetrics.

if nargin < 2, ctrl = []; end
if nargin < 3 || isempty(opts), opts = struct(); end

opts = localDefault(opts, 'tEnd',   abs(p.T) * 8);
opts = localDefault(opts, 'h',      abs(p.T) / 200);
opts = localDefault(opts, 'psi0',   0);
opts = localDefault(opts, 'r0',     0);
opts = localDefault(opts, 'delta0', 0);
opts = localDefault(opts, 'psiRef', 0);
opts = localDefault(opts, 'ctrlState', []);
opts = localDefault(opts, 'deltaOpenLoop', 0);

h = opts.h;
N = floor(opts.tEnd / h) + 1;

t        = (0:N-1)' * h;
psiLog   = zeros(N,1);
rLog     = zeros(N,1);
deltaLog = zeros(N,1);
cmdLog   = zeros(N,1);
refLog   = zeros(N,1);

x = [opts.psi0; opts.r0; opts.delta0];
ctrlState = opts.ctrlState;

refFcn = localToFcn(opts.psiRef);
olFcn  = localToFcn(opts.deltaOpenLoop);

for k = 1:N
    tk     = t(k);
    psiRef = refFcn(tk);

    if isempty(ctrl)
        deltaCmd = olFcn(tk);
    else
        [deltaCmd, ctrlState] = ctrl(tk, x(1), x(2), psiRef, ctrlState, h);
    end

    psiLog(k)   = x(1);
    rLog(k)     = x(2);
    deltaLog(k) = localAppliedDelta(x, deltaCmd, p);
    cmdLog(k)   = deltaCmd;
    refLog(k)   = psiRef;

    if k == N, break; end

    % RK4 (제어 지령은 스텝 구간 내 영차유지 — 실제 디지털 제어기와 동일)
    k1 = nomotoDerivative(x,                 deltaCmd, p);
    k2 = nomotoDerivative(x + (h/2) * k1,    deltaCmd, p);
    k3 = nomotoDerivative(x + (h/2) * k2,    deltaCmd, p);
    k4 = nomotoDerivative(x +  h    * k3,    deltaCmd, p);
    x  = x + (h/6) * (k1 + 2*k2 + 2*k3 + k4);

    % 타각 포화 (상태 클램프)
    if p.actuator.enable
        x(3) = max(-p.actuator.deltaMax, min(p.actuator.deltaMax, x(3)));
    end
end

out = struct( ...
    't',        t, ...
    'psi',      psiLog, ...
    'r',        rLog, ...
    'delta',    deltaLog, ...
    'deltaCmd', cmdLog, ...
    'psiRef',   refLog, ...
    'err',      ssa(refLog - psiLog), ...
    'p',        p, ...
    'opts',     opts);

end

% ------------------------------------------------------------------------
function s = localDefault(s, field, value)
if ~isfield(s, field) || isempty(s.(field))
    s.(field) = value;
end
end

function f = localToFcn(v)
if isa(v, 'function_handle')
    f = v;
else
    f = @(~) v;
end
end

function d = localAppliedDelta(x, deltaCmd, p)
if p.actuator.enable
    d = x(3);
else
    d = deltaCmd;
    if isfinite(p.actuator.deltaMax)
        d = max(-p.actuator.deltaMax, min(p.actuator.deltaMax, d));
    end
end
end
