function data = gen_data()
close all
clear
% Generate data for 2D radar to radar velocity calibration
%Set timespan and get normalized time
max_time = 20;
dt = 0.1; %s
data.dt = dt;
t = 0:dt:max_time;
data.t = t;
num_meas = length(t);
u = t/max_time;
dudt=1/max_time;

%Get Extrinsic Calibration values
data.t_V_AV = rand(2, 1) ;
data.theta_AV = angle_sub(2 * pi * rand);
data.t_V_BV = rand(2, 1);
data.theta_BV = angle_sub(2 * pi * rand);

%Generate cubic path
cubic_coeffs = [zeros(2, 1), 20 * [1; 1], 20 * ([-1; 0]), 50 * (rand(2, 1) - [0; 1])];
cubic_coeffs(:, 4) = [-cubic_coeffs(2, 3); cubic_coeffs(1, 3)];
path = cubic_coeffs(:, 1) + cubic_coeffs(:, 2) * u + cubic_coeffs(:, 3) * u.^2 + cubic_coeffs(:, 4) * u.^3;

%Get Radar measurements
dott_i_vi = dudt * (cubic_coeffs(:, 2) + 2 * cubic_coeffs(:, 3) * u + 3 * cubic_coeffs(:, 4) * u.^2);
data.theta_vi = atan2(dott_i_vi(2, :), dott_i_vi(1, :));
v_rot_mats = zeros(2 * num_meas, 2 * num_meas);
for i=1:num_meas
    v_rot_mats(2 * i - 1: 2 * i, 2 * i - 1: 2 * i) = theta2dcm(data.theta_vi(i));
end
data.v_vel = reshape(v_rot_mats * dott_i_vi(:), 2, num_meas);
ddott_i_vi =  dudt^2 * (2 * cubic_coeffs(:, 3) + 6 * cubic_coeffs(:, 4) * u);
accel_in_v = reshape(v_rot_mats * ddott_i_vi(:), 2, num_meas);
data.rot_vel_vi = accel_in_v(2, :)./data.v_vel(1, :);
data.A_meas = radar_meas(data.v_vel, data.rot_vel_vi, data.t_V_AV, data.theta_AV);
data.B_meas = radar_meas(data.v_vel, data.rot_vel_vi, data.t_V_BV, data.theta_BV);

%Verify dott_i_vi
test_dott_i_vi = diff(path, 1, 2)/dt;

%Verify rot_vel
test_rot_vel_vi = diff(data.theta_vi)/dt;

%Verify project_vel
theta_ai = data.theta_AV + data.theta_vi;
A_rot_mats = zeros(2 * num_meas, 2 * num_meas);
for i=1:num_meas
    A_rot_mats(2 * i - 1: 2 * i, 2 * i - 1: 2 * i) = theta2dcm(theta_ai(i));
end
t_i_Ai = reshape(transpose(v_rot_mats) * repmat(data.t_V_AV, num_meas, 1), 2, num_meas) + path;
dott_i_Ai = reshape(transpose(A_rot_mats) * data.A_meas(:), 2, num_meas);
test_dott_i_Ai = diff(t_i_Ai, 1, 2)/dt;
theta_bi = data.theta_BV + data.theta_vi;
B_rot_mats = zeros(2 * num_meas, 2 * num_meas);
for i=1:num_meas
    B_rot_mats(2 * i - 1: 2 * i, 2 * i - 1: 2 * i) = theta2dcm(theta_bi(i));
end
t_i_Bi = reshape(transpose(v_rot_mats) * repmat(data.t_V_BV, num_meas, 1), 2, num_meas) + path;

%Plot the path of the vehicle
figure
hold on
plot(path(1, :), path(2, :), 'b')
plot(t_i_Ai(1, :), t_i_Ai(2, :), 'r')
plot(t_i_Bi(1, :), t_i_Bi(2, :), 'g')
title("Unicycle Robot Path")
xlabel("X Position [m]")
ylabel("Y Position [m]")
legend("Robot Path", "Sensor A Path", "Sensor B Path")
return