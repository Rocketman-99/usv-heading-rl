function fig = plotHeadingResponse(outs, labels, ttl)
% PLOTHEADINGRESPONSE  침로 응답 시간 이력 플롯 (침로 / 선수각속도 / 타각).
%
%   fig = plotHeadingResponse(out)
%   fig = plotHeadingResponse({out1, out2}, {'baseline','RL'}, '비교')
%
%   HANDOFF.md 8장 3항: 성능 지표가 좋아도 타각이 진동하면 채택하지 않는다.
%   따라서 타각 이력을 항상 함께 그린다.

if ~iscell(outs), outs = {outs}; end
if nargin < 2 || isempty(labels)
    labels = arrayfun(@(i) sprintf('case %d', i), 1:numel(outs), 'UniformOutput', false);
end
if nargin < 3, ttl = 'Heading response'; end

r2d = 180/pi;
fig = figure('Name', ttl);

% --- 침로 ---
subplot(3,1,1); hold on; grid on;
plot(outs{1}.t, outs{1}.psiRef * r2d, 'k--', 'DisplayName', '\psi_{ref}');
for i = 1:numel(outs)
    plot(outs{i}.t, outs{i}.psi * r2d, 'LineWidth', 1.5, 'DisplayName', labels{i});
end
ylabel('\psi [deg]'); title(ttl); legend('Location','southeast');

% --- 선수각속도 ---
subplot(3,1,2); hold on; grid on;
for i = 1:numel(outs)
    plot(outs{i}.t, outs{i}.r * r2d, 'LineWidth', 1.5, 'DisplayName', labels{i});
end
ylabel('r [deg/s]');

% --- 타각 ---
subplot(3,1,3); hold on; grid on;
for i = 1:numel(outs)
    plot(outs{i}.t, outs{i}.delta * r2d, 'LineWidth', 1.5, 'DisplayName', labels{i});
end
p = outs{1}.p;
if p.actuator.enable && isfinite(p.actuator.deltaMax)
    yline_( p.actuator.deltaMax * r2d);
    yline_(-p.actuator.deltaMax * r2d);
end
ylabel('\delta [deg]'); xlabel('t [s]');

end

% ------------------------------------------------------------------------
function yline_(y)
% yline 은 구버전 MATLAB / Octave 에 없다.
xl = xlim;
plot(xl, [y y], 'r:', 'HandleVisibility', 'off');
xlim(xl);
end
