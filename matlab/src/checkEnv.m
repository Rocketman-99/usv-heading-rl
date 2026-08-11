function info = checkEnv()
% CHECKENV  MATLAB 환경 확인. HANDOFF.md 10장 3번 항목.
%
%   info = checkEnv()
%
%   출력된 내용을 docs/01_requirements.md 의 "3. 기술 스택 및 환경" 표에
%   그대로 옮겨 적는다. Claude Code 는 이 값을 추측해 채우지 않는다.

info = struct();

fprintf('==================================================\n');
fprintf(' usv-heading-rl 환경 확인\n');
fprintf('==================================================\n');

isOctave = exist('OCTAVE_VERSION', 'builtin') ~= 0;
info.isOctave = isOctave;

if isOctave
    info.release = ['GNU Octave ' version()];
    fprintf('실행 환경 : %s\n', info.release);
    fprintf('⚠ Octave 에서는 RL Toolbox / Simulink 를 쓸 수 없다.\n');
    fprintf('  1단계 플랜트 검증까지는 가능하나, 학습 루프는 MATLAB 또는\n');
    fprintf('  Python 대안이 필요하다 (HANDOFF.md 3장 참조).\n');
else
    v = ver('MATLAB');
    info.release = sprintf('MATLAB %s (%s)', v.Version, v.Release);
    fprintf('실행 환경 : %s\n', info.release);
end

fprintf('플랫폼    : %s\n', computer('arch'));

% --- 툴박스 확인 ------------------------------------------------------
targets = { ...
    'Simulink',                          'Simulink'
    'Reinforcement Learning Toolbox',    'RL_Toolbox'
    'Control System Toolbox',            'Control_Toolbox'
    'Optimization Toolbox',              'Optimization_Toolbox'
    'Deep Learning Toolbox',             'Neural_Network_Toolbox'
    'Statistics and Machine Learning Toolbox', 'Statistics_Toolbox'
    };

fprintf('\n%-42s %-10s %-10s\n', '툴박스', '설치', '라이선스');
fprintf('%s\n', repmat('-', 1, 64));

info.toolboxes = struct();
for i = 1:size(targets, 1)
    name    = targets{i,1};
    feature = targets{i,2};

    installed = false;
    verStr    = '';
    if ~isOctave
        vv = ver(localVerKey(name));
        installed = ~isempty(vv);
        if installed, verStr = vv(1).Version; end
    end

    licensed = false;
    if ~isOctave
        try
            licensed = license('test', feature) == 1;
        catch
            licensed = false;
        end
    end

    fprintf('%-42s %-10s %-10s %s\n', name, ...
        localYN(installed), localYN(licensed), verStr);

    key = matlab.lang.makeValidName(name);
    info.toolboxes.(key) = struct('installed', installed, ...
                                  'licensed',  licensed, ...
                                  'version',   verStr);
end

% --- 프로젝트 경로 ----------------------------------------------------
fprintf('\n프로젝트 함수 접근성:\n');
needed = {'nomotoParams', 'nomotoDerivative', 'simulateHeading', ...
          'pidHeading', 'headingMetrics', 'ssa'};
allOk = true;
for i = 1:numel(needed)
    ok = exist(needed{i}, 'file') == 2;
    allOk = allOk && ok;
    fprintf('  %-20s %s\n', needed{i}, localYN(ok));
end
if ~allOk
    fprintf('\n⚠ 경로가 등록되지 않았다. 저장소 루트에서 startup_project 를 실행할 것.\n');
end
info.pathOk = allOk;

fprintf('\n==================================================\n');
fprintf('위 결과를 docs/01_requirements.md 3장에 기록한다.\n');
fprintf('==================================================\n');

end

% ------------------------------------------------------------------------
function s = localYN(tf)
if tf, s = 'yes'; else, s = 'NO'; end
end

function k = localVerKey(name)
% ver() 는 툴박스 짧은 이름을 받는다.
switch name
    case 'Simulink',                                 k = 'simulink';
    case 'Reinforcement Learning Toolbox',           k = 'rl';
    case 'Control System Toolbox',                   k = 'control';
    case 'Optimization Toolbox',                     k = 'optim';
    case 'Deep Learning Toolbox',                    k = 'nnet';
    case 'Statistics and Machine Learning Toolbox',  k = 'stats';
    otherwise,                                       k = name;
end
end
