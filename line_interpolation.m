function [interpolated_data2] = line_interpolation(data1, data2)

% data is struct including the time_stamps and estimated velocity 

% data1 is the reference for interpolation -- in this case: Radar 5
% data2 is the data to be interpolated -- in this case: Radar 3

% return the interpolated radar2 data as interpolated_data2

data1_size = size(data1.mag_vel);
data2_size = size(data2.mag_vel);

data1.type = ones(data1_size);
data2.type = zeros(data2_size);

final_data_size = data1_size;
data_total = struct();
data_total.mag_vel = cat(2, data1.mag_vel, data2.mag_vel);
data_total.type = cat(2, data1.type, data2.type);
data_total.time_stamp = cat(2, data1.time_stamp, data2.time_stamp);

[data_total.sorted_mag_vel,idx]=sort(data_total.mag_vel);
data_total.sorted_type = data_total.type(idx);
data_total.sorted_time_stamp = data_total.time_stamp(idx);
data_total.total_size = data1_size + data2_size;

a = 1;
b = 1;
i = 1;
j = 1;

for i = 1:data1_size
    
    while (data1.mag_vel(a) < data2.mag_vel(b))
        a = a+1; % mark as the starting location for data1 --> all data from a will be picked 
    end 
    target_point = data1.mag_vel(a);
    target_time = data1.time_stamp(a);
    b1 = data2.mag_vel(b);
    time1 = data2.time_stamp(b);

    while (data1.mag_vel(a) > data2.mag_vel(b+1))
        b = b+1;
        b1 = data2.mag_vel(b);
        time1 = data2.time_stamp(b);
    end 
    b2 = data2.mag_vel(b+1);
    time2 = data2.time_stamp(b+1);

    k = (b2 - b1)/(time2 - time1);
    interpolated_data2() = b1 + k*(target_time - time1);

end 

% if data_total.sorted_type(1) == 1
%     while data_total.sorted_type(i) == 1
%         b1 = data_total.sorted_mag_vel(i);
%         time2_1 = data_total.sorted_time_stamp(i);
%         i = i + 1;
%     end
% 
%     
%     while data_total.sorted_type(i) == 1
%         b2 = data_total.sorted_mag_vel(i);
%         time2_2 = data_total.sorted_time_stamp(i);
%         i = i + 1;
%     end
%     
% else
%     b1 = data_total.sorted_mag_vel(i);
%     time2_1 = data_total.sorted_time_stamp(i);
%     i = i + 1; 
%     while data_total.sorted_type(i) == 1
%          b2 = data_total.sorted_mag_vel(i);
%          time2_2 = data_total.sorted_time_stamp(i);
%          i = i + 1;
%     end
% 
% end 

k = (data2.mag_vel(j+1) - data2.mag_vel(j))/(data2.time_stamp(j+1) - data2.time-stamp(j));
interpolated_data2(a) = data2.mag_vel(j) + k*(data1.time_stamp(i) - data2.time_stamp(j));

end






