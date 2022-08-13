function [C,dim,error] = uncertainty_cal(x,y,A)

% inputs:
% x: the estimated ego velocity vector 
% y: measured radical velocities vector
% A: measured range matrix

% outputs:
% C: uncertainty 

[dim,a] = size(y);
error = A*x - y;
C = (error'*error)*inv(A'*A)/(dim-2);
error = error' * error;
end
