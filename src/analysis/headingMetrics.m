function m = headingMetrics(out, opts)
% HEADINGMETRICS  침로 제어 성능 지표 (HANDOFF.md 7장).
%
%   m = headingMetrics(out)
%   m = headingMetrics(out, opts)
%
%   입력 out 은 simulateHeading 의 출력. 계단 목표침로 응답을 전제로 한다.
%
%   opts (선택):
%     .band        정정 시간 판정 대역, 계단 크기 대비 비율   기본 0.05 (= ±5%)
%     .ssFrac      정상상태 구간 정의. 시뮬레이션 뒷쪽 몇 %   기본 0.30 (= 마지막 30%)
%     .stepSize    계단 크기 [rad]. 생략 시 psiRef 로부터 추정
%
%   ⚠ band 와 ssFrac 은 "무엇을 정상상태로 보고 무엇을 정정으로 볼 것인가"라는
%     검증 조건이다. HANDOFF.md 5장에 따라 이는 작업자가 결정할 항목이며,
%     여기 기본값은 논의를 위한 출발점일 뿐이다. 확정 시 docs/decisions.md 에 기록.
%
%   출력 m (모든 각도 단위는 deg):
%     .rmsErrSS      정상상태 침로 오차 RMS      [deg]
%     .rmsErrAll     전 구간 침로 오차 RMS       [deg]
%     .tEnter        ±band 대역 최초 진입 시각   [s]   (HANDOFF 7장의 "정정 시간" 정의)
%     .tSettle       이후 이탈 없이 유지되는 정착 시각 [s]  (표준 정의)
%     .overshoot     최대 초과 각도              [deg]
%     .overshootPct  계단 크기 대비 오버슈트     [%]
%     .rudderTravel  타각 총 이동량 int|ddelta/dt|dt [deg]  (HANDOFF 7장 "러더 사용량")
%     .rudderEffort  int delta^2 dt              [deg^2*s]  (보조 지표)
%     .deltaMaxAbs   최대 타각 절댓값            [deg]
%     .satFrac       타각 포화 시간 비율         [-]
%     .band, .ssFrac, .stepSize  사용된 판정 조건 (재현성 기록용)
%
%   tEnter 와 tSettle 을 모두 낸다. HANDOFF 7장은 "±5% 이내 진입까지 소요 시간"
%   으로 정의하지만, 진입 후 다시 이탈하는 응답에서는 이 값이 성능을 과대평가한다.
%   두 값이 크게 다르면 진동이 남아 있다는 신호다.
%
%   See also simulateHeading, plotHeadingResponse.

if nargin < 2 || isempty(opts), opts = struct(); end
if ~isfield(opts, 'band')    || isempty(opts.band),    opts.band    = 0.05; end
if ~isfield(opts, 'ssFrac')  || isempty(opts.ssFrac),  opts.ssFrac  = 0.30; end

t    = out.t;
err  = out.err;                       % ssa(psiRef - psi) [rad]
psi  = out.psi;
ref  = out.psiRef;
dlt  = out.delta;

r2d = 180/pi;

% --- 계단 크기 --------------------------------------------------------
if isfield(opts, 'stepSize') && ~isempty(opts.stepSize)
    stepSize = opts.stepSize;
else
    stepSize = ssa(ref(end) - psi(1));
end
A = abs(stepSize);
if A < eps
    warning('headingMetrics:zeroStep', ...
        '계단 크기가 0 이다. 정정 시간/오버슈트는 정의되지 않는다.');
end

% --- 침로 오차 RMS ----------------------------------------------------
N     = numel(t);
iSS   = max(1, floor(N * (1 - opts.ssFrac)));
m.rmsErrSS  = sqrt(mean(err(iSS:end).^2)) * r2d;
m.rmsErrAll = sqrt(mean(err.^2))          * r2d;

% --- 정정 시간 --------------------------------------------------------
tol    = opts.band * A;
inBand = abs(err) <= tol;

iEnter = find(inBand, 1, 'first');
if isempty(iEnter)
    m.tEnter = NaN;                    % 대역에 한 번도 못 들어옴
else
    m.tEnter = t(iEnter);
end

iOut = find(~inBand, 1, 'last');       % 마지막으로 대역 밖이었던 시점
if isempty(iOut)
    m.tSettle = t(1);
elseif iOut == N
    m.tSettle = NaN;                   % 끝까지 대역 밖 → 정착 실패
else
    m.tSettle = t(iOut + 1);
end

% --- 오버슈트 ---------------------------------------------------------
% 목표를 지나친 양. 계단 방향 기준으로 부호를 맞춘 뒤 최대 초과분을 취한다.
if A < eps
    m.overshoot    = 0;
    m.overshootPct = 0;
else
    s        = sign(stepSize);
    excess   = s * ssa(psi - ref);     % > 0 이면 목표를 지나침
    ov       = max(0, max(excess));
    m.overshoot    = ov * r2d;
    m.overshootPct = 100 * ov / A;
end

% --- 러더 사용량 ------------------------------------------------------
% int |ddelta/dt| dt = 타각이 실제로 움직인 총 거리. 사다리꼴 적분과 무관하게
% 변화량의 합으로 정확히 계산된다.
m.rudderTravel = sum(abs(diff(dlt))) * r2d;
m.rudderEffort = trapz(t, (dlt * r2d).^2);
m.deltaMaxAbs  = max(abs(dlt)) * r2d;

if out.p.actuator.enable && isfinite(out.p.actuator.deltaMax)
    m.satFrac = mean(abs(dlt) >= out.p.actuator.deltaMax * (1 - 1e-6));
else
    m.satFrac = 0;
end

% --- 판정 조건 기록 ---------------------------------------------------
m.band     = opts.band;
m.ssFrac   = opts.ssFrac;
m.stepSize = stepSize * r2d;

end
