function tbl = printMetrics(ms, labels)
% PRINTMETRICS  headingMetrics 결과를 비교표로 출력한다.
%
%   printMetrics(m)
%   tbl = printMetrics({m1, m2}, {'baseline PID','RL gains'})
%
%   HANDOFF.md 8장 1항: 베이스라인 대비 비교표는 반드시 결과물에 포함한다.
%   반환값 tbl 은 markdown 표 문자열이라 docs/ 에 그대로 붙여 넣을 수 있다.

if ~iscell(ms), ms = {ms}; end
if nargin < 2 || isempty(labels)
    labels = arrayfun(@(i) sprintf('case %d', i), 1:numel(ms), 'UniformOutput', false);
end

rows = { ...
    '침로 오차 RMS (정상상태)', 'rmsErrSS',     'deg',     '%.4f'
    '침로 오차 RMS (전 구간)',  'rmsErrAll',    'deg',     '%.4f'
    '대역 진입 시간',            'tEnter',       's',       '%.2f'
    '정착 시간',                 'tSettle',      's',       '%.2f'
    '오버슈트',                  'overshoot',    'deg',     '%.3f'
    '오버슈트',                  'overshootPct', '%',       '%.2f'
    '러더 사용량 int|d.delta|',  'rudderTravel', 'deg',     '%.2f'
    '최대 타각',                 'deltaMaxAbs',  'deg',     '%.2f'
    '포화 시간 비율',            'satFrac',      '-',       '%.3f'
    };

lines = {};
hdr = '| 지표 | 단위 |';
sep = '|---|---|';
for i = 1:numel(labels)
    hdr = [hdr ' ' labels{i} ' |'];   %#ok<AGROW>
    sep = [sep '---|'];               %#ok<AGROW>
end
lines{end+1} = hdr;
lines{end+1} = sep;

for k = 1:size(rows,1)
    line = sprintf('| %s | %s |', rows{k,1}, rows{k,3});
    for i = 1:numel(ms)
        v = ms{i}.(rows{k,2});
        if isnan(v)
            line = [line ' n/a |'];                             %#ok<AGROW>
        else
            line = [line ' ' sprintf(rows{k,4}, v) ' |'];        %#ok<AGROW>
        end
    end
    lines{end+1} = line;                                        %#ok<AGROW>
end

tbl = strjoin(lines, sprintf('\n'));

fprintf('\n%s\n\n', tbl);
fprintf('판정 조건: 대역 ±%.1f%% (계단 %.1f deg), 정상상태 구간 = 뒤쪽 %.0f%%\n\n', ...
    ms{1}.band * 100, ms{1}.stepSize, ms{1}.ssFrac * 100);

end
