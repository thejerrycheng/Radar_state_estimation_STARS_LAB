function [err, J, R] = batch_g_accel(meas, x)
% Measurement Model
% x is the state vector [v_0; omega_0; ... ;v_k; omega_k ; theta_t; theta_BA]
%Initialize Parameters
window_size = size(meas, 2);
unit_t = [cos(x(end - 1, 1)); sin(x(end - 1,1))];
J=zeros(4 * window_size, size(x, 1));
err = zeros( 4 * window_size, 1);
R = zeros(4 * window_size);
for i=1:window_size    
    %Measurement Errors
    err(4 * i - 3:4 * i - 2, 1) = meas(1:2, i) - x(6 * i - 5: 6 * i - 4, 1);
    err(4 * i - 1:4 * i, 1) = meas(7:8, i) - radar_meas(x(6 * i - 5: 6 * i - 4, 1) , x(6 * i - 3, 1), unit_t, x(end, 1));
    
    %Jacobian
    J(4 * i - 3:4 * i - 2, 6 * i - 5: 6 * i - 4) = - eye(2);
    J_sub = -construct_G([x(6 * i - 5: 6 * i - 3, 1); 0; x(end-1, 1); x(end, 1)]);
    J(4 * i - 1:4 * i, 6 * i - 5: 6 * i - 4) = J_sub(3:4, 1:2);
    J(4 * i - 1:4 * i, 6 * i - 3) = J_sub(3:4, 3);
    J(4 * i - 1:4 * i, end - 1) = J_sub(3:4, end - 1);
    J(4 * i - 1:4 * i, end) = J_sub(3:4, end);
    
    %Uncertainty
    R(4 * i - 3:4 * i - 2, 4 * i - 3:4 * i - 2) = inv(reshape(meas(3:6, i), 2, 2));
    R(4 * i - 1:4 * i, 4 * i - 1:4 * i) = inv(reshape(meas(9:12, i), 2, 2));
end
return