function meas = g(x)
% Measurement Model
% x is the state vector [v; omega; theta_t; theta_BA]
% noise is the noise vetor for the motion model
unit_t = [cos(x(5, 1)); sin(x(5,1))];
meas = [x(1:2, 1); radar_meas(x(1:2, 1) , x(3, 1), unit_t, x(6, 1))];

return