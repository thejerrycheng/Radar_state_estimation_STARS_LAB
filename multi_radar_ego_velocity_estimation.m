clc;
clear all;
clear

%% importing radar data 
[odom_time, rotated_linear, ...
    rotated_x, rotated_y, rotated_z, ...
    radar1_struct,radar2_struct,radar3_struct,radar4_struct,radar5_struct]...
    = import_radar_data...
    ('/Users/jerrycheng/Desktop/Summer Research /Radar Data/filtered_radar_pointcloud2_data/mid1_radar1.bag');

%% estimating ego velocities: 
[data5] = ego_vel_estimator(radar5_struct, odom_time, rotated_linear, rotated_x, rotated_y, rotated_z);
[data3] = ego_vel_estimator_noisy(radar3_struct, odom_time, rotated_linear, rotated_x, rotated_y, rotated_z);
% [vel_x1,vel_y1,mag_vel1,total_frame1,uncertainty1,time_stamp1, time1, C1] = ego_vel_estimator_noisy(radar1_struct);
% [vel_x2,vel_y2,mag_vel2,total_frame2,uncertainty2,time_stamp2, time2, C2] = ego_vel_estimator_noisy(radar2_struct);
% [vel_x4,vel_y4,mag_vel4,total_frame4,uncertainty4,time_stamp4, time4, C4] = ego_vel_estimator_noisy(radar4_struct);


[C3_out ,C5_out, vel_x3_out, vel_y3_out, vel_x5_out, vel_y5_out] = interpretation_solver(C3, C5, vel_x3, vel_y3, vel_x5, vel_y5);
% [C1_out ,C5_out, vel_x1_out, vel_y1_out, vel_x5_out, vel_y5_out] = interpretation_solver(C3, C5, vel_x3, vel_y3, vel_x5, vel_y5);
% [C3_out ,C5_out, vel_x3_out, vel_y3_out, vel_x5_out, vel_y5_out] = interpretation_solver(C3, C5, vel_x3, vel_y3, vel_x5, vel_y5);
% [C3_out ,C5_out, vel_x3_out, vel_y3_out, vel_x5_out, vel_y5_out] = interpretation_solver(C3, C5, vel_x3, vel_y3, vel_x5, vel_y5);

%% plotting the output data
% t5 = linspace(0, double(max(odom_time)), double(total_frame5(1)));

% figure 
% tiledlayout(3,1);

% nexttile
% plot(odom_time, rotated_x);
% hold on; 
% plot(t5, vel_x5);
% plot(t5, vel_x5 + uncertainty5(:, 1)', '-r', t5, vel_x5 - uncertainty5(:, 1)', '-r')
% title('velocity at the x direction');

% nexttile
% plot(odom_time, rotated_y);
% hold on; 
% plot(t5, vel_y5);

% plot(t5, vel_y5 + uncertainty5(:, 2)', '-r', t5, vel_y5 - uncertainty5(:, 2)', '-r')
% title('velocity at the y direction');

% nexttile
% plot (t5,sqrt(vel_x5.^2+vel_y5.^2));
% hold on;
% plot (odom_time,sqrt(rotated_x.^2+rotated_y.^2));
% title("magnitude of the velocity comparision")
% xlabel('time');
% ylabel('velocity in m/s');

plot_function(odom_time, rotated_x, rotated_y, vel_x5,vel_y5,mag_vel5,total_frame5,uncertainty5,time_stamp5, time5, C5)
plot_function(odom_time, rotated_x, rotated_y, vel_x3,vel_y3,mag_vel3,total_frame3,uncertainty3,time_stamp3, time3, C3)
% plot_function(odom_time, rotated_x, rotated_y, vel_x1,vel_y1,mag_vel1,total_frame1,uncertainty1,time_stamp1, time1, C1)
% plot_function(odom_time, rotated_x, rotated_y, vel_x2,vel_y2,mag_vel2,total_frame2,uncertainty2,time_stamp2, time2, C2)
% plot_function(odom_time, rotated_x, rotated_y, vel_x4,vel_y4,mag_vel4,total_frame4,uncertainty4,time_stamp4, time4, C4)


