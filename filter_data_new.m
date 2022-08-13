function [y_new,A_new] = filter_data_new(y, A, range_x, range_y, interval)
%Input: unfiltered y and A 
% where A is normalized vector 

%Output: filtered y and A using manual filtering method

   
%% filtering based on the radical velocity
vel_indices = abs(y) < 10; % set max doppler velocity as 12
y = y(vel_indices);
A = A(vel_indices, :); % the initially filtered A 
range_x_filtered = range_x(vel_indices);
range_y_filtered = range_y(vel_indices);

%% filtering based on field of view
A_tan = A(:,1)./A(:,2);
A_theta = atan(A_tan);
% hyperparameter fov_tan
fov_tan = sqrt(3)/3;
fov_indices = abs(A_tan) > fov_tan;
y = y(fov_indices);
A = A(fov_indices, :);
range_x_filtered = range_x(fov_indices);
range_y_filtered = range_y(fov_indices);

%     range_x{i} = range_x{i}(fov_indices);
%     range_y{i} = range_y{i}(fov_indices);

%% filtering based on the divided areas 
A_tan_filtered = range_x_filtered./range_y_filtered;
A_theta_filtered = atan(A_tan_filtered); 

%% filtering based on the divided areas 

% hyperparameter max_num
% hyperparameter internal

% interval = 5; % interval in degrees of interval for each divided area 
                % interval*pi/180 in radians 
max_num = 1;
number_of_area = (pi - 2*atan(fov_tan))/(interval*pi/180);
 
point_size = size(A);

down_bound = -90 + fov_tan*180/pi;
up_bound = down_bound + interval;

for num = 1:number_of_area % ititerate for number of area times 
    
    valid_num = 0;
    down_bound = down_bound + interval;
    up_bound = down_bound + interval;

    for j = 1:point_size 
        if (A_theta_filtered(j) > down_bound) && (A_theta_filtered(j) < up_bound) && valid_num < 1
            A_filtered(num,:) = A(j,:);
            y_filtered(num,:) = y(j,:);
            valid_num = 1;
            break;
        end 
    end 

end 

y_new = y_filtered;
A_new = A_filtered;

end 
