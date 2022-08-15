function coloured_point_cloud(range_x, range_x_in, range_y, range_y_in, error, error_in, time_frame)

num_points = size(error);
num_inlier_points = size(error_in);

a = 1;
b = 1;
c = 1;
d = 1;
e = 1;
f = 1;
g = 1;

range_x_1 = [];
range_y_1 = [];
range_x_2 = [];
range_y_2 = [];
range_x_3 = [];
range_y_3 = [];
range_x_4 = [];
range_y_4 = [];
range_x_5 = [];
range_y_5 = [];
range_x_6 = [];
range_y_6 = [];
range_x_7 = [];
range_y_7 = [];

% range_x_in_1 = [];
% range_y_in_1 = [];
% range_x_in_2 = [];
% range_y_in_2 = [];
% range_x_in_3 = [];
% range_y_in_3 = [];
% range_x_in_4 = [];
% range_y_in_4 = [];
% range_x_in_5 = [];
% range_y_in_5 = [];
% range_x_in_6 = [];
% range_y_in_6 = [];
% range_x_in_7 = [];
% range_y_in_7 = [];

for i = 1:num_points(1)

    if error(i) < 0.05
        range_x_1(a) = range_x(i);
        range_y_1(a) = range_y(i);
        a = a+1;

    elseif error(i) < 0.1
        range_x_2(b) = range_x(i);
        range_y_2(b) = range_y(i);
        b = b+1;

    elseif error(i) < 0.5
        range_x_3(c) = range_x(i);
        range_y_3(c) = range_y(i);
        c = c+1;

    elseif error(i) < 1
        range_x_4(d) = range_x(i);
        range_y_4(d) = range_y(i);
        d = d+1;

    elseif error(i) < 2
        range_x_5(e) = range_x(i);
        range_y_5(e) = range_y(i);
        e = e+1;

    elseif error(i) < 5
        range_x_6(f) = range_x(i);
        range_y_6(f) = range_y(i);
        f = f+1;

    else 
        range_x_7(g) = range_x(i);
        range_y_7(g) = range_y(i);
        g = g+1;
    end 
end 

% [sortedError sortIndexes] = sort(abs(error));
% range_xs = range_x(sortIndexes);
% range_ys = range_y(sortIndexes);
% % fprintf('this is time at %d', time_frame)
% figure
% cmap = jet(length(error));
% scatter(range_xs, range_ys, 50, cmap, 'filled')
% grid on;
% title(sprintf('Point Cloud Color Graded at Time Frame %d', time_frame));
% % Enlarge figure to full screen.
% set(gcf, 'units','normalized','outerposition',[0 0 1 1]);
% % Give a name to the title bar.
% set(gcf,'name','Time frame', 'numbertitle','off');
% colormap(jet(length(error)));
% % a = colorbar;
% % ylabel(a,'Power (db)','FontSize',16,'Rotation',270);
% % hColourbar.Label.Position(1) = 3;
% labels = sortedError;
% lcolorbar(labels,'fontweight','bold');

figure
color = jet(7);
p1 = plot(range_x_1, range_y_1, '.','Color',color(1,:),'MarkerSize',20);
hold on;
p2 = plot(range_x_2, range_y_2, '.','Color',color(2,:),'MarkerSize',20);
hold on;
p3 = plot(range_x_3, range_y_3, '.','Color',color(3,:),'MarkerSize',20);
hold on;
p4 = plot(range_x_4, range_y_4, '.','Color',color(4,:),'MarkerSize',20);
hold on;
p5 = plot(range_x_5, range_y_5, '.','Color',color(5,:),'MarkerSize',20);
hold on;
plot(range_x_6, range_y_6, '.','Color',color(6,:),'MarkerSize',20);
hold on;
plot(range_x_7, range_y_7, '.','Color',color(7,:),'MarkerSize',20);
hold on;
plot(range_x_in, range_y_in, 'o', 'MarkerSize',20);
hold on; 

title(sprintf('Point Cloud Color Graded at Time Frame %d', time_frame));
colormap(jet(7))
set(gca,'CLim',[4 11])
labels = {'0.05','0.1','0.5','1','2','5','inf'};
lcolorbar(labels,'fontweight','bold');

% p1.MarkerEdgeColor = [1 0 0];
% p1.MarkerFaceColor = [1 0 0];
% p1.MarkerSize = 20;
% p2.MarkerEdgeColor = [1 0.2 0];
% p2.MarkerFaceColor = [1 0.2 0];
% p2.MarkerSize = 20;
% p3.MarkerEdgeColor = [1 0.4 0];
% p3.MarkerFaceColor = [1 0.4 0];
% p3.MarkerSize = 20;
% p4.MarkerEdgeColor = [1 0.6 0];
% p4.MarkerFaceColor = [1 0.6 0];
% p4.MarkerSize = 20;
% p5.MarkerEdgeColor = [1 0.8 0];
% p5.MarkerFaceColor = [1 0.8 0];
% p5.MarkerSize = 20;
% p6.MarkerEdgeColor = [1 1 0];
% p6.MarkerFaceColor = [1 1 0];
% p6.MarkerSize = 20;
% p7.MarkerEdgeColor = [1 1 0.2];
% p7.MarkerFaceColor = [1 1 0.2];
% p7.MarkerSize = 20;

end










