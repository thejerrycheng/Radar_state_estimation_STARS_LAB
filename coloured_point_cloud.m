function coloured_point_cloud(range_x, range_y, error)

num_points = size(error);

[sortedError sortIndexes] = sort(error);
range_xs = range_x(sortIndexes);
range_ys = range_y(sortIndexes);

figure
cmap = jet(length(error));
scatter(range_xs, range_ys, 50, cmap, 'filled')
grid on;
title('Points where color depends on error magnitude -- red means more error; blue means less error', ...
  'FontSize', 10);
% Enlarge figure to full screen.
% set(gcf, 'units','normalized','outerposition',[0 0 1 1]);
% Give a name to the title bar.
set(gcf,'name','Colour-graded point cloud data based on the error value','numbertitle','off');
colormap(cmap);


% plot(range_x_1, range_y_1, 'o');
% hold on;
% plot(range_x_2, range_y_2, 'o');
% hold on;
% plot(range_x_3, range_y_3, 'o');
% hold on;
% plot(range_x_4, range_y_4, 'o');
% hold on;
% plot(range_x_5, range_y_5, 'o');
% hold on;
% plot(range_x_6, range_y_6, 'o');
% hold on;
% plot(range_x_7, range_y_7, 'o');
% hold on;


end

