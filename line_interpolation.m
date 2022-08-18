function [time, interpolated_data1, interpolated_data2] = line_interpolation(data1, data2)

% data is struct including the time_stamps and estimated velocity 

% data1 is the reference for interpolation -- in this case: Radar 5
% data2 is the data to be interpolated -- in this case: Radar 3

% return the interpolated radar2 data as interpolated_data2


[m, data1_size] = size(data1.mag_vel);
[m, data2_size] = size(data2.mag_vel);

data1.type = ones(1,data1_size);
data2.type = zeros(1,data2_size);

data_total = struct();
data_total.mag_vel = cat(2, data1.mag_vel, data2.mag_vel);
data_total.type = cat(2, data1.type, data2.type);
data_total.time_stamp = cat(2, data1.time_stamp, data2.time_stamp);

[data_total.sorted_mag_vel,idx]=sort(cell2mat(data_total.time_stamp));
data_total.sorted_type = data_total.type(idx);
data_total.sorted_time_stamp = data_total.time_stamp(idx);
data_total.total_size = data1_size;

a = 1;
b = 0;
i = 1;
j = 1;

% edge case for the starting edge: 
if cell2mat(data1.time_stamp(1)) < cell2mat(data2.time_stamp(1))
    while (cell2mat(data1.time_stamp(a)) < cell2mat(data2.time_stamp(1)))
        a = a+1;
        fprintf('starting edge case')
    end 
end 
% edge case for the ending edge: 
if cell2mat(data1.time_stamp(data1_size)) > cell2mat(data2.time_stamp(data2_size))
    while (cell2mat(data1.time_stamp(data1_size - b)) > cell2mat(data2.time_stamp(data2_size)))
        b = b+1;
        fprintf('ending edge case \n' )
    end
end 

interpolated_data1 = data1.mag_vel(a:(data1_size - b));
time = cell2mat(data1.time_stamp(a:(data1_size - b)));

% the total generated points after subtracting the two edge cases: 
data_total.total_size = data_total.total_size - a - b + 1;

data1_index = a;
data2_index = 1;
fprintf('finished the initial processing \n')
% iterate the interpolated points 
for i = 1:(data_total.total_size)
    
    target_point = data1.mag_vel(data1_index);
    target_time = cell2mat(data1.time_stamp(data1_index));
    b1 = data2.mag_vel(data2_index);
    b2 = (data2.mag_vel(data2_index+1));
    time1 = cell2mat(data2.time_stamp(data2_index));
    time2 = cell2mat(data2.time_stamp(data2_index+1));

    while (cell2mat(data1.time_stamp(data1_index)) > cell2mat(data2.time_stamp(data2_index + 1)))
        data2_index = data2_index + 1;
        b1 = (data2.mag_vel(data2_index));
        b2 = (data2.mag_vel(data2_index+1));
        time1 = cell2mat(data2.time_stamp(data2_index));
        time2 = cell2mat(data2.time_stamp(data2_index+1));
    end 

    k = (b2 - b1)/(time2 - time1);
    interpolated_data2(i) = b1 + k*(target_time - time1);
    data1_index = data1_index+1;
    time_diff1 = target_time - time1;
    time_diff2 = time2 - target_time; 

end 

plotend






