function startup_project()
% STARTUP_PROJECT  프로젝트 경로 등록.
%
%   저장소 루트에서 실행한다.
%
%       >> startup_project
%
%   MATLAB / GNU Octave 양쪽에서 동작한다.

here = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(here, 'src')));

fprintf('[usv-heading-rl] src/ 경로 등록 완료: %s\n', here);
fprintf('[usv-heading-rl] 다음 단계:\n');
fprintf('    checkEnv                      %% 환경 확인 (docs/01_requirements.md 3장 채우기)\n');
fprintf('    run_step_response_check       %% 1단계-4: Nomoto 계단 응답 확인\n');
fprintf('    run_baseline_tuning           %% 1단계-5: 베이스라인 PID 수동 튜닝 (작업자 직접)\n');

end
