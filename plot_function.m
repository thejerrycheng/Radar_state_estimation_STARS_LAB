function plot_function(odom_time_stamps, rotated_x, rotated_y, data)

% t = linspace(0, double(max(odom_time)), double(total_frame(1)));
%%
figure 
tiledlayout(6,1);

nexttile
plot (cell2mat(odom_time_stamps),sqrt(rotated_x.^2+rotated_y.^2));
hold on;
plot(cell2mat(data.time_stamp), data.mag_vel);
title("magnitude of the velocity comparision")
xlabel('time');
ylabel('velocity in m/s');

nexttile
plot (cell2mat(odom_time_stamps),sqrt(rotated_x.^2+rotated_y.^2));
hold on;
plot(cell2mat(data.time_stamp), data.hampel);
title("magnitude of the velocity comparision")
xlabel('time');
ylabel('velocity in m/s');

%%



nexttile
plot(cell2mat(odom_time_stamps), rotated_x);
hold on; 
plot(cell2mat(data.time_stamp), data.x_velmat);
plot(cell2mat(data.time_stamp), data.x_velmat + data.uncertainty(:, 1)', '-r', cell2mat(data.time_stamp), data.x_velmat - data.uncertainty(:, 1)', '-r')
title('velocity at the x direction');

nexttile
plot(cell2mat(odom_time_stamps), rotated_x);
hold on; 
plot(cell2mat(data.time_stamp), data.hampel_x);
plot(cell2mat(data.time_stamp), data.hampel_x + data.uncertainty(:, 1)', '-r', cell2mat(data.time_stamp), data.hampel_x - data.uncertainty(:, 1)', '-r')
title('velocity at the x direction');

%%
nexttile
plot(cell2mat(odom_time_stamps), rotated_y);
hold on; 
plot(cell2mat(data.time_stamp), data.hampel_y);
plot(cell2mat(data.time_stamp), data.hampel_y + data.uncertainty(:, 2)', '-r', cell2mat(data.time_stamp), data.hampel_y - data.uncertainty(:, 2)', '-r')
title('velocity at the y direction');

nexttile
plot(cell2mat(odom_time_stamps), rotated_y);
hold on; 
plot(cell2mat(data.time_stamp), data.y_velmat);
plot(cell2mat(data.time_stamp), data.y_velmat + data.uncertainty(:, 2)', '-r', cell2mat(data.time_stamp), data.y_velmat - data.uncertainty(:, 2)', '-r')
title('velocity at the y direction');

% nexttile
% plot (t,sqrt(vel_x.^2+vel_y.^2));
% hold on;
% plot (odom_time,sqrt(rotated_x.^2+rotated_y.^2));
% title("magnitude of the velocity comparision")
% xlabel('time');
% ylabel('velocity in m/s');
% plot (cell2mat(odom_time_stamps),sqrt(rotated_x.^2+rotated_y.^2));

end