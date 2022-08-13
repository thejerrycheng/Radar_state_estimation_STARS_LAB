function [err, J, P] = batch_prior_accel(x_prior, x, std_devs)
%Prior Error
err = zeros(8, 1);
err(1:6) = x_prior(1:6) - x(1:6);
err(7:8) = x_prior(end-1:end) - x(end-1:end);

%Jacobian
J = zeros(8, size(x, 1));
J(1:6, 1:6) = -eye(6);
J(7:8, end-1:end) = -eye(2);

%Uncertianty
P = diag([1./std_devs(1:6), 1./std_devs(7:8)]);
end