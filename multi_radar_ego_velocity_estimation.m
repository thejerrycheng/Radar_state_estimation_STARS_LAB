clc;
clear all;
clear

%% importing radar data 
[odom_time, rotated_linear, ...
    rotated_x, rotated_y, rotated_z, ...
    radar1_struct,radar2_struct,radar3_struct,radar4_struct,radar5_struct]...
    = import_radar_data...
    ('/Users/jerrycheng/Desktop/Summer Research /Radar Data/filtered_radar_pointcloud2_data/east1_radar1.bag');

%% estimating ego velocities: 
[vel_x5,vel_y5,mag_vel5,total_frame5,uncertainty5,time_stamp5, time5, C5] = ego_vel_estimator(radar5_struct);
[vel_x3,vel_y3,mag_vel3,total_frame3,uncertainty3,time_stamp3, time3, C3] = ego_vel_estimator_noisy(radar3_struct);


[C3_out ,C5_out, vel_x3_out, vel_y3_out, vel_x5_out, vel_y5_out] = interpretation_solver(C3, C5, vel_x3, vel_y3, vel_x5, vel_y5);

%% plotting the output data
t5 = linspace(0, double(max(odom_time)), double(total_frame5(1)));

plot_function(odom_time, rotated_x, rotated_y, vel_x3,vel_y3,mag_vel3,total_frame3,uncertainty3,time_stamp3, time3, C3)

figure 
tiledlayout(3,1);

nexttile
plot(odom_time, rotated_x);  
hold on; 
plot(t5, vel_x5);
plot(t5, vel_x5 + uncertainty5(:, 1)', '-r', t5, vel_x5 - uncertainty5(:, 1)', '-r')
title('velocity at the x direction');

nexttile
plot(odom_time, rotated_y);
hold on; 
plot(t5, vel_y5);

plot(t5, vel_y5 + uncertainty5(:, 2)', '-r', t5, vel_y5 - uncertainty5(:, 2)', '-r')
title('velocity at the y direction');

nexttile
plot (t5,sqrt(vel_x5.^2+vel_y5.^2));
hold on;
plot (odom_time,sqrt(rotated_x.^2+rotated_y.^2));
title("magnitude of the velocity comparision")
xlabel('time');
ylabel('velocity in m/s');
