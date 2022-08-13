function [err, J, P] = batch_prior(x_prior, x, std_devs, opt)
%Prior Error
err = zeros(5, 1);
err(1:3) = x_prior(1:3) - x(1:3);
err(4:5) = x_prior(end-1:end) - x(end-1:end);
if opt
    %Jacobian
    J=sparse(5, size(x, 1));
    J(1:3, 1:3) = -speye(3);
    J(4:5, end-1:end) = -speye(2);
    
    %Uncertianty
    P = spdiags([1./std_devs(1:3), 1./std_devs(4:5)]', 0, 5, 5);
end
end