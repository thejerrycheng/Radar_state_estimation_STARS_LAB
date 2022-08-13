function [x_velmat,y_velmat,mag_vel,total_frame,uncertainty, time_stamp, time, C] = ego_vel_estimator_noisy(radar_struct)

total_frame = size(radar_struct);

%% data captured by radar1: 
for i = 1:total_frame(1)
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

end 

j = 1;

for i = 1:total_frame(1)
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
    [x{i}, invalid_frames{i},range_x_in{i}, range_y_in{i}, vel_in{i}]= ransac_solver_noisy(y_new{i},A_new{i});
    
    x_vel_init{i} = x{i}(1);
    y_vel_init{i} = x{i}(2);
    x_velmat_init = cell2mat(x_vel_init);
    y_velmat_init = cell2mat(y_vel_init);
    mag_vel_init = sqrt(x_velmat_init.^2+y_velmat_init.^2);

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
        mag_vel_new = sqrt(x_velmat.^2+y_velmat.^2);
        j = j+1;
    end 
end

total_frame = j-1;
uncertainty = zeros(total_frame,2);

for j = 1:total_frame

    %% Uncertainty Calculation:
    [C{j},dim{j},error{j}] = uncertainty_cal(x_new{j},y_new_new{j},A_new_new{j});
    uncertainty(j, :) = 3 *sqrt(diag(C{j}));

end 

time = time_new;
time_stamp = time_stamp_new;
mag_vel = mag_vel_new;

end
