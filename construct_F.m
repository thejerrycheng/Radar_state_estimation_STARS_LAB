function F = construct_F(dt)
F = eye(6);
F(3, 4) = dt;
end