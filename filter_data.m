function [y_new,A_new] = filter_data(y, A, range_x, range_y)
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
max_num = 12;
 
point_size = size(A);
a = 0;
b = 0;
c = 0;
d = 0;
e = 0;
f = 0;
k = 1;

    for j = 1:point_size
    
        if (A_theta_filtered(j) >= -pi/2) && (A_theta_filtered(j) < (-pi/2+pi/6)) && a < max_num
            a = a+1;
            A_filtered(k,:) = A(j,:);
            y_filtered(k,:) = y(j,:);
            k = k+1;
        end 
    
        if (A_theta_filtered(j) >= -pi/2 + pi/6) && (A_theta_filtered(j) < (-pi/2 + pi/3)) && b < max_num
            b = b+1;
            A_filtered(k,:) = A(j,:);
            y_filtered(k,:) = y(j,:);
            k = k+1;
        end
    
        if (A_theta_filtered(j) >= -pi/2 + pi/3) && (A_theta_filtered(j) < (-pi/2 + pi/2)) && c < max_num
            c = c+1;
            A_filtered(k,:) = A(j,:);
            y_filtered(k,:) = y(j,:);
            k = k+1;
        end
    
        if (A_theta_filtered(j) >= -pi/2 + pi/2) && (A_theta_filtered(j) < (-pi/2 + 2*pi/3)) && d < max_num
            d = d+1;
            A_filtered(k,:) = A(j,:);
            y_filtered(k,:) = y(j,:);
            k = k+1;
        end
    
        if (A_theta_filtered(j) >= -pi/2 + 2*pi/3) && (A_theta_filtered(j) < (-pi/2 + 5*pi/6)) && e < max_num
            e = e+1;
            A_filtered(k,:) = A(j,:);
            y_filtered(k,:) = y(j,:);
            k = k+1;
        end

        if (A_theta_filtered(j) >= -pi/2 + 5*pi/6) && (A_theta_filtered(j) < (-pi/2 + pi)) && f < max_num
            f = f+1;
            A_filtered(k,:) = A(j,:);
            y_filtered(k,:) = y(j,:);
            k = k+1;
        end 

    end 

    y_new = y_filtered;
    A_new = A_filtered;
end 
