function proj_vel = radar_meas(vel, rot_vel, trans, theta)
num_meas = size(vel, 2);
rot_vel_skew = zeros(2 * num_meas, 2 * num_meas);
for i=1:num_meas
    rot_vel_skew(2 * i - 1: 2 * i, 2 * i - 1: 2 * i) = skew2d(rot_vel(i));
end
proj_vel = theta2dcm(theta) * (reshape(rot_vel_skew * repmat(trans, num_meas, 1), 2, num_meas) + vel);
end