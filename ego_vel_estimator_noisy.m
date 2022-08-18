function [data] = ego_vel_estimator_noisy(radar_struct, odom_time_stamps, rotated_x, rotated_y)

total_frame = size(radar_struct);

%% construct the struct data varialble for this radar 
data = struct();
data.total_frame = total_frame(1);
data.index = [1:1:data.total_frame];

j = 1;

%% data captured by radar1: 
for i = 1:data.total_frame
    height{i} = radar_struct{i}.Height;
    width{i} = radar_struct{i}.Width;
    pointstep{i} = radar_struct{i}.PointStep; 
    row_step{i} = radar_struct{i}.RowStep;
    time_stamp{i} = double(radar_struct{i}.Header.Stamp.Sec)+1e-9*double(radar_struct{i}.Header.Stamp.Nsec);
    % x1,y1 for range; vel_rad for the measured velocity 
    % x1{i} means the point clouds captured at the specific time instance 
    range_x{i} = rosReadField(radar_struct{i},"x");
    range_y{i} = rosReadField(radar_struct{i},"y");
    vel_rad{i} = rosReadField(radar_struct{i},"vel_rad"); 

    %% Getting y Vector and Normalized A Matrix
    A_vector{i} = cell2mat({range_x{i}, range_y{i}});
    A_vector_normalized{i} = A_vector{i}./repmat(vecnorm(A_vector{i}, 2, 2), 1, 2); % nomalization 
    y_vector{i} = -vel_rad{i}; 
    y{i} = y_vector{i}; % the initially filtered y
    A{i} = A_vector_normalized{i};
    time{i} = time_stamp{i} - min(cell2mat(time_stamp));
   
    %% Initial Manual Filtering 
    [y_new{i}, A_new{i}] = filter_data(y{i}, A{i}, range_x{i}, range_y{i});
   
    %% Solving the Equation Using RANSAC 
    [x{i}, invalid_frames{i}, range_x_in{i}, range_y_in{i}, vel_in{i}, error{i}, error_in{i}]= ransac_solver_noisy(y_new{i},A_new{i}, range_x{i}, range_y{i});
    
    x_vel_init{i} = x{i}(1);
    y_vel_init{i} = x{i}(2);
    x_velmat_init = cell2mat(x_vel_init);
    y_velmat_init = cell2mat(y_vel_init);
    mag_vel_init = sqrt(x_velmat_init.^2+y_velmat_init.^2);

    %% getting rid of the non solution time frames 
    if mag_vel_init(i) ~= 0 && mag_vel_init(i) < 7
        x_vel{j} = x_vel_init{i};
        y_vel{j} = y_vel_init{i};
        time_stamp_new{j} = time_stamp{i};
        time_new{j} = time{i};
        x_velmat = cell2mat(x_vel);
        y_velmat = cell2mat(y_vel);
        x_new{j} = x{i};
        y_new_new{j} = y_new{i};
        A_new_new{j} = A_new{i};
        error_new{j} = error{i};
        error_in_new{j} = error_in{i};
        range_x_in_new{j} = range_x_in{i};
        range_y_in_new{j} = range_y_in{i};
        range_x_new{j} = range_x{i};
        range_y_new{j} = range_y{i};
        mag_vel_new = sqrt(x_velmat.^2+y_velmat.^2);
        j = j+1;
    end 
end

time = time_new;
time_stamp = time_stamp_new;
mag_vel = mag_vel_new;

data.range_x = range_x; %OG
data.range_y = range_y; %OG
data.range_x_in = range_x_in; % after the ransac filter 
data.range_y_in = range_y_in; % after the ransac filter 
data.range_x_in_new = range_x_in_new;
data.range_y_in_new = range_y_in_new;
data.vel_rad = vel_rad; %OG
data.vel_in = vel_in; % after the ransac filter 
data.y = y; 
data.y_new = y_new; % after the filter function 
data.A = A;
data.A_new = A_new; % after the filter function 
data.x_velmat = x_velmat;
data.y_velmat = y_velmat;
data.mag_vel = mag_vel; 
data.error = error; % before the filtering -- added zero solution time frames
data.error_in = error_in; 
data.error_new = error_new; % after ransac filter -- with only the solutions 
data.error_in_new = error_in_new; 
% data.fit_indices = fit_indices;

total_frame = j-1;
uncertainty = zeros(total_frame,2);

for j = 1:total_frame
    %% Uncertainty Calculation:
    [C{j},dim{j},error{j}] = uncertainty_cal(x_new{j},y_new_new{j},A_new_new{j});
    uncertainty(j, :) = 3 *sqrt(diag(C{j}));
end 

data.C = C;
data.time_stamp = time_stamp;
data.time = time;
data.uncertainty = uncertainty;

% added hampel filter to further eliminate the outliers 

windowSize = 125;
numMedians = 1;
[data.hampel,outliers]=hampel(data.mag_vel,windowSize,numMedians);
[data.hampel_x,outliers_x]=hampel(data.x_velmat,windowSize,numMedians);
[data.hampel_y,outliers_y]=hampel(data.y_velmat,windowSize,numMedians);

plot_function(odom_time_stamps, rotated_x, rotated_y, data);

end






