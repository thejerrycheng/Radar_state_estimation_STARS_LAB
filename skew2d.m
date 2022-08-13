function mat = skew2d(val)
mat = zeros(2,2);
mat(1, 2) = -val;
mat(2, 1) = val;
return