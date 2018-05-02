function theta = inverseKinematics2R(xytraj)
%Both input and output are 2*n matrix.

L1 = 1;
L2 = 1;
n = size(xytraj, 2);
theta = zeros(2, n);
l = norm(xytraj(:, i));

for i = 1: n
    theta(2, i) = pi - acos((L1^2 + L2^2 - l^2 ) / 2 / L1 / L2);
    theta(1, i) = atan(xytraj(2, i) / xytraj(1, i)) - acos((L1^2 + l^2 - L2^2) / 2 / L1 / l);
end
end