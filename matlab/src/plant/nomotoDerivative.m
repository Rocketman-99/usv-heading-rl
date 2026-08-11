function xdot = nomotoDerivative(x, deltaCmd, p)
% NOMOTODERIVATIVE  1차 Nomoto 모델 + 구동기 동특성의 상태미분.
%
%   xdot = nomotoDerivative(x, deltaCmd, p)
%
%   상태:
%     x(1) = psi    선수각        [rad]
%     x(2) = r      선수각속도    [rad/s]
%     x(3) = delta  실제 타각     [rad]   (p.actuator.enable = false 이면 x(3) = deltaCmd)
%
%   입력:
%     deltaCmd  지령 타각 [rad]  (p.inputType = 'yaw_moment' 이면 요 모멘트 [N*m])
%     p         nomotoParams 가 반환한 구조체
%
%   동역학:
%     psidot   = r
%     rdot     = (-r + K * delta) / T
%     deltadot = sat( (deltaCmd - delta) / Tdelta , +-deltaRateMax )
%
%   타각 포화는 적분기(simulateHeading)에서 상태를 클램프해 처리한다.
%   여기서는 포화 경계 밖으로 나가는 미분을 0 으로 만들어 안티와인드업만 담당한다.
%
%   See also nomotoParams, simulateHeading.

psi   = x(1);  %#ok<NASGU>  (미분에 직접 쓰이지 않음)
r     = x(2);
delta = x(3);

act = p.actuator;

if act.enable
    deltadot = (deltaCmd - delta) / act.Tdelta;

    % 변화율 제한
    if abs(deltadot) > act.deltaRateMax
        deltadot = sign(deltadot) * act.deltaRateMax;
    end

    % 포화 경계에서 바깥으로 미는 미분은 차단
    if (delta >=  act.deltaMax && deltadot > 0) || ...
       (delta <= -act.deltaMax && deltadot < 0)
        deltadot = 0;
    end

    deltaEff = delta;
else
    % 구동기 동특성 없음: 지령이 즉시 반영
    deltadot = 0;
    deltaEff = deltaCmd;
end

rdot   = (-r + p.K * deltaEff) / p.T;
psidot = r;

xdot = [psidot; rdot; deltadot];

end
