function [err, F, Q] = batch_f(x, std_dev_mm, opt)
%% Motion model for 2D radar to radar calibration
state_length = (length(x) - 2)/3;
%Error
err = reshape(diff(reshape(x(1:end-2), 3, []), 1, 2), [], 1);
if opt
%Jacobian
F = spdiags([-ones(size(err,1), 1), ones(size(err, 1), 1)], [0,3], length(err), length(x));
%Uncertianty
Q = spdiags(repmat(1./std_dev_mm', state_length - 1, 1), 0, size(err,1), size(err,1));
end
end