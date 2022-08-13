function x_new = add_accel_to_state(x, t)
%Reshape into state timesteps
x_temp = reshape(x(1:end-2), 3, []);
accels = diff(x_temp, 1, 2)./repmat(diff(t), 3, 1);
accels = [accels, accels(:, end)];
x_new = [reshape([x_temp; accels], [], 1); x(end-1:end)];
end