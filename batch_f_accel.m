function [err, F, Q] = batch_f_accel(t, x, cov)
%% Motion model for 2D radar to radar calibration
state_length = (length(x) - 2)/6;
delta_t = diff(t);

%Compute Errors
F = zeros((state_length - 1) * 6, size(x,1));
Q = zeros((state_length - 1) * 6, (state_length - 1) * 6);
states = reshape(x(1:end-2), 6, []); %break states according to time
for i=1:state_length - 1
    F_single = [eye(3), delta_t(i) * eye(3); zeros(3), eye(3)];
    F(6 * i -5: 6 * i, 6 * i -5: 6 * i) = -F_single;
    F(6 * i -5: 6 * i, 6 * i + 1: 6 * i + 6) = eye(6);
    Q(6 * i -5: 6 * i, 6 * i -5: 6 * i) = F_single * diag(1./cov) * F_single';
end
err = F * x;
end