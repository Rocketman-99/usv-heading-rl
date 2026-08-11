function a = ssa(angle)
% SSA  Smallest signed angle. 각도를 (-pi, pi] 로 사상한다.
%
%   a = ssa(angle)   angle [rad], 배열 가능
%
%   침로 오차를 계산할 때 반드시 거쳐야 한다.
%   예: psiRef = 179 deg, psi = -179 deg 의 오차는 358 deg 가 아니라 -2 deg 다.

a = mod(angle + pi, 2*pi) - pi;

% mod 결과가 정확히 -pi 인 경우를 +pi 로 올려 (-pi, pi] 를 유지
a(a == -pi) = pi;

end
