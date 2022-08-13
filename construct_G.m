function G = construct_G(x)
% x is the state vector [v; omega; alpha; theta_t; theta_BA]
G = zeros(4,6);
unit_t = [cos(x(5, 1)); sin(x(5,1))];
G(:, 1:2) = [eye(2); theta2dcm(x(6, 1))];
G(3:4, 3) = theta2dcm(x(6, 1)) * skew2d(1) * unit_t;
G(3:4, 5) = theta2dcm(x(6, 1))* skew2d(x(3, 1)) * [-sin(x(5,1)); cos(x(5, 1))];
G(3:4, 6) = theta2J(x(6, 1)) * (skew2d(x(3, 1)) * unit_t + x(1:2,1));
end