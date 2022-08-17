function [odom_time,odom_time_stamps, rotated_linear, ...
    rotated_x, rotated_y, rotated_z, ...
    radar1_struct,radar2_struct,radar3_struct,radar4_struct,radar5_struct]...
    = import_radar_data(file_path)

    data = rosbag (file_path);
   
    radar1 = select(data,'Topic','/RadarDetection/Pointcloud2/Near_1');
    radar2 = select(data,'Topic','/RadarDetection/Pointcloud2/Near_2');
    radar3 = select(data,'Topic','/RadarDetection/Pointcloud2/Near_3');
    radar4 = select(data,'Topic','/RadarDetection/Pointcloud2/Near_4');
    radar5 = select(data,'Topic','/RadarDetection/Pointcloud2/Near_5');
    odom = select(data,'Topic','/pioneer_sensors/EKF_Localization_RS232/filteredodometry');
    
    radar1_struct = readMessages(radar1,'DataFormat','struct');
    radar2_struct = readMessages(radar2,'DataFormat','struct');
    radar3_struct = readMessages(radar3,'DataFormat','struct');
    radar4_struct = readMessages(radar4,'DataFormat','struct');
    radar5_struct = readMessages(radar5,'DataFormat','struct');
    odom_struct = readMessages(odom,'DataFormat','struct');
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Odom Data Capture %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    [total_odom_frame,col] = size(odom_struct);
    
    for i = 1:total_odom_frame
        linear_x{i} = odom_struct{i}.Twist.Twist.Linear.X;
        linear_y{i} = odom_struct{i}.Twist.Twist.Linear.Y;
        linear_z{i} = odom_struct{i}.Twist.Twist.Linear.Z;
    
        pose_x{i} = odom_struct{i}.Pose.Pose.Position.X;
        pose_y{i} = odom_struct{i}.Pose.Pose.Position.Y;
        pose_z{i} = odom_struct{i}.Pose.Pose.Position.Z;
        
        orientation_x{i} = odom_struct{i}.Pose.Pose.Orientation.X;
        orientation_y{i} = odom_struct{i}.Pose.Pose.Orientation.Y;
        orientation_z{i} = odom_struct{i}.Pose.Pose.Orientation.Z;
        orientation_w{i} = odom_struct{i}.Pose.Pose.Orientation.W;
    
        odom_time_stamps{i} = double(odom_struct{i}.Header.Stamp.Sec)+1e-9*double(odom_struct{i}.Header.Stamp.Nsec);
        odom_time{i} = odom_time_stamps{i} - odom_time_stamps{1};
    end 
    
    gt_x = cell2mat(linear_x);
    gt_y = cell2mat(linear_y);
    pose_x = cell2mat(pose_x);
    pose_y = cell2mat(pose_y);
    orientation_x = cell2mat(orientation_x)';
    orientation_y = cell2mat(orientation_y)';
    orientation_z = cell2mat(orientation_z)';
    orientation_w = cell2mat(orientation_w)';
    quat = [orientation_w, orientation_x, orientation_y, orientation_z];
    quat = quaternion(quat);
    og_x = cell2mat(linear_x);
    og_y = cell2mat(linear_y);
    og_z = cell2mat(linear_z);
    linear = cell2mat([linear_x; linear_y; linear_z]);
    
    %% convert to rotational matrix
    rotational_matrix = quat2rotm(quat);
    
    %% getting the 2D rotational matrix -- to be normalized 
    %% getting the correct velocity in respect to the inertia frame 
    for i = 1:total_odom_frame
        rotational_matrix_two_d(:,:,i) = rotational_matrix(1:2,1:2,i);
        rotated_linear(:,i) = rotational_matrix(:,:,i)'*linear(:,i);
        rotated_x(i) = rotated_linear(1,i);
        rotated_y(i) = rotated_linear(2,i);
        rotated_z(i) = rotated_linear(3,i);
    end 
    
    odom_time = cell2mat(odom_time);
    [a, odom_size] = size(odom_time);
   

end
