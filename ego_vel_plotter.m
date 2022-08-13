function [] = ...
    ego_vel_plotter(odom_time,rotated_x,rotated_y, total_frame, x_velmat, y_velmat, uncertainty)

t = linspace(0, double(max(odom_time)), double(total_frame(1)));

figure 
tiledlayout(3,1);

nexttile
plot(odom_time, rotated_x);
hold on; 
plot(t, x_velmat);
plot(t, x_velmat + uncertainty(:, 1)', '-r', t, x_velmat - uncertainty(:, 1)', '-r')
title('velocity at the x direction');

nexttile
plot(odom_time, rotated_y);
hold on; 
plot(t, y_velmat);

plot(t, y_velmat + uncertainty(:, 2)', '-r', t, y_velmat - uncertainty(:, 2)', '-r')
title('velocity at the y direction');

nexttile
plot (t,sqrt(x_velmat.^2+y_velmat.^2));
hold on;
plot (odom_time,sqrt(rotated_x.^2+rotated_y.^2));
title("magnitude of the velocity comparision")
xlabel('time');
ylabel('velocity in m/s');

end
