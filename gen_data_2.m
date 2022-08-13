function data = gen_data_2()
close all
clear
% Generate data for 2D radar to radar velocity calibration
%Set timespan and get normalized time
max_time = 5.5;
dt = 1/14; %s
data.dt = dt;
t = 0:dt:max_time;
data.t = t;
num_meas = length(t);

A_meas_mag = 10;
rot_vel_mag = 1;

%data.A_meas = [A_meas_mag * ones(1, size(t,2)); zeros(size(t))];
data.A_meas = [sin(2 *t) + 5; 0.25 * cos(0.5 *t) + 1];
data.theta_BA = 2 * pi * rand - pi;
theta_temp = pi *rand;
data.t_A_BA = 2 *[cos(theta_temp);
    sin(theta_temp)];
data.rot_vel_vi = rot_vel_mag * (cos(2 * t) + 2)./max_time;
data.B_meas = radar_meas(data.A_meas, data.rot_vel_vi, data.t_A_BA, data.theta_BA);
return