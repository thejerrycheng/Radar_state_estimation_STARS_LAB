function [A, y, A_in, y_in] = synthesis_radar_data(x, num)
% A being the range matrix 
% y being the doppler velocity matrix
% x being the ego velocity ground truth x = [x_x; x_y]
% number being the number of data to be generated 

total_num = num;
inliner_num = 0.95*total_num;
outliner_num = total_num - inliner_num;

global sigma_v
sigma_v = 0.01; % m/s

ego_vel = x; 
ego_vel_x = ego_vel(1);
ego_vel_y = ego_vel(2);

y_out = 5*rand([outliner_num 1]);
A_out = 5*rand([outliner_num 2]);

A_in = rand([inliner_num 2]); % 2D unit vector matrix
A_in = A_in./repmat(vecnorm(A_in, 2, 2), 1, 2); % nomalization 

y_in = A_in * ego_vel + sigma_v * randn(inliner_num,1); % the measured doppler velocity 

y = [y_in; y_out];
A = [A_in; A_out];

ix = randperm(total_num);

y = y(ix,:);
A = A(ix,:);

% y = y_total(randperm(size(y_total, 1)), :);
% A = A_total(randperm(size(A_total, 1)), :);

