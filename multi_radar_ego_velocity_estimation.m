clc;
clear all;
clear

%% importing radar data 
[odom_time, odom_time_stamps, rotated_linear, ...
    rotated_x, rotated_y, rotated_z, ...
    radar1_struct,radar2_struct,radar3_struct,radar4_struct,radar5_struct]...
    = import_radar_data...
    ('/Users/jerrycheng/Desktop/Summer Research /Radar Data/filtered_radar_pointcloud2_data/mid1_radar1.bag');

%% estimating ego velocities: 
[data5] = ego_vel_estimator(radar5_struct, odom_time_stamps, rotated_x, rotated_y);
[data3] = ego_vel_estimator_noisy(radar3_struct, odom_time_stamps, rotated_x, rotated_y);

%% plot the estimated velocity vs the ground truth
plot_function(odom_time_stamps, rotated_x, rotated_y, data5);
plot_function(odom_time_stamps, rotated_x, rotated_y, data3);


