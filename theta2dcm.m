function dcm = theta2dcm(theta)
dcm = [cos(theta), sin(theta);...
-sin(theta), cos(theta)];
return