%% Radar 2 Radar Extrinsic Cal
close all
clear

% addpath('/Users/jerrycheng/Desktop/Summer Research /STARS Lab/matlab_source_code/updated');

%% Controls
window_length = 2520; % 14*60*3 -- 14Hz sampling rate; 60 seconds; 3 mins
initialization_type = 2; %gt, noisy, all ones

[odom_time, odom_time_stamps, rotated_linear, ...
    rotated_x, rotated_y, rotated_z, ...
    radar1_struct,radar2_struct,radar3_struct,radar4_struct,radar5_struct]...
    = import_radar_data...
    ('/Users/jerrycheng/Desktop/Summer Research /Radar Data/filtered_radar_pointcloud2_data/east1_radar1.bag');

%% Pre Data Generation Initialization
meas_sigma = 0.1;
R_A = meas_sigma^2 * diag(ones(2 * window_length, 1));
R_B = meas_sigma^2 * diag(ones(2 * window_length, 1));
% data = gen_data();
data = gen_data_2();

[vel_x5,vel_y5,mag_vel5,total_frame5,uncertainty5,time_stamp5, time5, C5] = ego_vel_estimator(radar5_struct);
[vel_x3,vel_y3,mag_vel3,total_frame3,uncertainty3,time_stamp3, time3, C3] = ego_vel_estimator_noisy(radar3_struct);

[C3_out ,C5_out, vel_x3_out, vel_y3_out, vel_x5_out, vel_y5_out] = interpretation_solver(C3, C5, vel_x3, vel_y3, vel_x5, vel_y5);


% data.A_meas = cat(1,vel_x3_out', vel_y3_out', C3_out);
% data.B_meas = cat(1,vel_x5_out', vel_y5_out', C5_out);

data.A_meas = cat(1,vel_x3_out', vel_y3_out');
data.B_meas = cat(1,vel_x5_out', vel_y5_out');

data.theta_AV = 1.52;
data.theta_BV = 1.52;
data.rot_vel_vi = ones(1,8000);  
data.t_V_BV = [-0.5;0.5];
data.t_V_AV = [-1.68;1.106];
data.t = linspace(0,600,8000); % 600 for the time amount and 8000 for the total number of frames 
data.dt = 600/8000;
data.theta_vi = data.A_meas(1,:)./data.A_meas(2,:);% -- how to get theta_vi here 


%% Post Data Generation Initialization

%Initial State
%t_A_BA = theta2dcm(data.theta_AV) * (data.t_V_BV - data.t_V_AV);
theta_t_true = angle_sub(atan2(data.t_A_BA(2,1), data.t_A_BA(1,1)));
%theta_BA_true= angle_sub(data.theta_BV - data.theta_AV);
theta_BA_true= data.theta_BA;
theta_BA_start = theta_BA_true + pi/3 * (rand - 0.5);
std_dev_prior = [1, 1, 1, 120/180 * pi, 120/180 * pi];
std_dev_prior = std_dev_prior.^2;
std_dev_mm = 10 * [1, 1, 1];
std_dev_B = [meas_sigma^2, meas_sigma^2];

%Measurements
noisy_poses = [data.A_meas;...
    repmat(meas_sigma^2 * [1; 0; 0; 1], 1, size(data.A_meas, 2));...
    data.B_meas;...
    repmat(meas_sigma^2 * [1; 0; 0; 1], 1, size(data.A_meas, 2))];
     noisy_poses = noisy_poses + [meas_sigma * randn([2, size(data.A_meas, 2)]);...
                                     zeros(4, size(data.A_meas, 2));
                                 meas_sigma * randn([2, size(data.A_meas, 2)]);
                                 zeros(4, size(data.A_meas, 2))];

%Initialize prior state
theta_BA_start = median(cross([noisy_poses(1:2, :)./vecnorm(noisy_poses(1:2, :), 2, 1); zeros(1, size(noisy_poses, 2))], [noisy_poses(7:8, :)./vecnorm(noisy_poses(7:8, :), 2, 1); zeros(1, size(noisy_poses, 2))]), 2);
theta_BA_start = theta_BA_start(3);
rot_vel_start = vecnorm(theta2dcm(theta_BA_start)' * noisy_poses(7:8, 1:window_length) - noisy_poses(1:2, 1:window_length), 2, 1);
x = [reshape([noisy_poses(1:2, 1:window_length);rot_vel_start(1:window_length)], [], 1);...
    theta_t_true + pi/3 * (rand - 0.5);...
    theta_BA_start];

% x = [reshape([noisy_poses(1:2, 1:window_length)+ meas_sigma * randn([2, window_length]);rand(1, window_length)], [], 1);...
%     theta_t_true + pi/4.5 * (rand - 0.5);...
%     theta_BA_true + pi/4.5 * (rand - 0.5)];

% x = [reshape([noisy_poses(1:2, 1:window_length); data.rot_vel_vi(1:window_length) * norm(data.t_A_BA)], [], 1);...
%      theta_t_true;...
%      theta_BA_true];

init_state = [x(1:3); x(end-1:end)];

%Storage for estimated states
x_store = zeros(5, length(data.t) - window_length + 1);
x_store(:, 1) = [x(1:2, 1); x(3 , 1); x(end-1:end, 1)];
P_store = zeros(5, length(data.t) - window_length + 1);

%Define current measurement window
current_meas_window = noisy_poses(:, 1:window_length);
% eigvalues = [];
%Get errors and Jacobians
for j = 1:length(data.t) - window_length
    state_size = (length(x) - 2)/3;
    lm_param = 10^5;
    for i = 1:1000
        [prior_err, J_prior, P] = batch_prior(init_state, x, std_dev_prior, true);
        [motion_err, F, Q] = batch_f(x, std_dev_mm, true);
        [meas_err, G, R_B] = batch_g(current_meas_window, x, true);
        H = [G];
        err = [meas_err];
        Q_total = blkdiag(R_B);
        cost=@(x) [batch_g(current_meas_window, x, false)]'*Q_total * [batch_g(current_meas_window, x, false)];
        A = H' * Q_total *  H;
        %A = A + lm_param * eye(size(A,1));
        b = H' * Q_total * err;
%         if j > 1000
%             A_schur = A(4:end, 4:end) - A(4:end, 1:3) * inv(A(1:3, 1:3)) * A(1:3, 4:end);
%             b_schur = b(4:end) - A(4:end, 1:3) * inv(A(1:3, 1:3)) * b(1:3);
%             del_x = A_schur\b_schur;
%             x(4:end) = x(4:end) - 0.5 * del_x;
%         else
        del_x = A\b;
        fline =@(a) cost(x - a * del_x);
        [a, new_cost] = fminsearch(fline, 0);
        x = x - min(a, 1) * del_x;
        if x(end-1) < 0
           x(end - 1) = x(end - 1) + pi;
           x = diag([repmat([1, 1, -1], 1, (size(x, 1) - 2)/3), 1, 1])*x;
        end
        
%         end
        prev_cost = err'* Q_total * err;
%         rho = (prev_cost - new_cost)/ (min(a, 1) *del_x' * (lm_param * del_x + H' * Q_total*err));
%         if rho < 0.9
%             lm_param = min([11 * lm_param, 10^7]);
%         else
%             x = x - min(a, 1) * del_x;
%             if i> 1
%                 test_vec = [max(a * del_x./x) < 1e-5, max(b) < 1e-5 , new_cost/(size(err, 1) - size(x, 1) + 1) < 1e-10];
%                 if sum(test_vec) > 0
%                     fprintf("Stopping run " + j + " of " + (length(data.t) - window_length) +"\n")
%                     test_vec
%                     break
%                 end
%             end
%             lm_param = max([lm_param/9, 10^(-7)]);
%         end
        test_vec = [max(a * del_x./x) < 1e-5, (prev_cost - new_cost)/prev_cost < 1e-5,  prev_cost - new_cost < 1e-5];
        if sum(test_vec) > 0
%             fprintf("Stopping run " + j + " of " + (length(data.t) - window_length) +"\n")
%             test_vec
            break
        end
        %[norm(del_x), err'* Q_total * err]
        x(end) = x(end) - (x(end) > pi) * 2 * pi;
        x(end) = x(end) + (x(end) < -pi) * 2 * pi;
        %Print Info
        %fprintf("Previous Cost: " + prev_cost + " | lm_parameter: " + lm_param + " | rho: " + rho +  "\n")
%         fprintf("Previous Cost: " + prev_cost + "\n")
    end
    %Add noise to state
    P_temp = spdiags(inv(H' * Q_total *  H), 0);
    %x = x + 6 * P_temp .* randn(size(P_temp, 1), 1);
    
    %Expand
    x = [x(1:end - 2); noisy_poses(1:2, window_length + j); x(end - 2); x(end - 1:end)];
    init_state = [x(1:3); x(end - 1:end)];
    current_meas_window = [current_meas_window, noisy_poses(:, window_length + j)];
    
    %Contract
    if j > 1
        x= x(4:end);
        current_meas_window = current_meas_window(:, 2:end);
    end

    % Store relevant values
    x_store(:, j + 1) = [x(1:3, 1); x(end-1:end, 1)];
    P_store(:, j + 1) = [P_temp(1:3, 1); P_temp(end-1:end, 1)];
    
    %Update priors
    std_dev_prior = [P_temp(1:3)', P_temp(end-1:end, 1)'];
    
end

plot_t = data.t(1:length(data.t) - window_length);
title_names = ["Ego Velocity", "Scaled Rotational Velocity", "Translation Direction", "Rotation"];
for i=1:4
    switch i
        case 1
            figure
            subplot(2, 1, 1)
            hold on
            plot(0, x_store(i, 1) - data.A_meas(1, 1), 'ok')
            plot(plot_t, x_store(i, 2:end) - data.A_meas(1, 1:length(data.t) - window_length), 'b')
            plot(plot_t, noisy_poses(1, 1:length(data.t) - window_length) - data.A_meas(1, 1:length(data.t) - window_length), 'g')
            plot(plot_t, 3 * sqrt(P_store(i, 2:end)), '-r')
            plot(plot_t, -3 * sqrt(P_store(i, 2:end)), '-r')
            title(title_names(i))
            ylabel("X-Velocity Error [m/s]")
            hold off
            subplot(2, 1, 2)
            hold on
            plot(0, x_store(i + 1, 1)- data.A_meas(2, 1), 'ok')
            plot(plot_t, x_store(i+1, 2:end) - data.A_meas(2, 1:length(data.t) - window_length) , 'b')
            plot(plot_t, noisy_poses(2, 1:length(data.t) - window_length) - data.A_meas(2, 1:length(data.t) - window_length), 'g')
            plot(plot_t, 3 * sqrt(P_store(i+1, 2:end)), '-r')
            plot(plot_t, -3 * sqrt(P_store(i+1, 2:end)), '-r')
            ylabel("Y-Velocity Error [m/s]")
            xlabel("Time [s]")
            hold off
            legend("Initial Guess", "Estimated Error","Raw Data", "3-Sigma bound")
        case 2
            figure
            hold on
            title(title_names(i))
            plot(0, x_store(3, 1)- data.rot_vel_vi(1) * norm(data.t_A_BA), 'ok')
            plot(plot_t, x_store(3, 2:end) - data.rot_vel_vi(1:length(data.t) - window_length) * norm(data.t_A_BA), 'b')
            plot(plot_t, 3 * sqrt(P_store(3, 2:end)), '-r')
            plot(plot_t, -3 * sqrt(P_store(3, 2:end)), '-r')
            ylabel("Scaled Rotational Velocity Error [rad/s]")
            xlabel("Time [s]")
            hold off
            legend("Initial Guess", "Estimated Error", "3-Sigma bound")
        case 3
            figure
            hold on
            title(title_names(i))
            plot(0, angle_sub(x_store(4, 1)- theta_t_true), 'ok')
            plot(plot_t, angle_sub(x_store(4, 2:end) - theta_t_true * ones(size(x_store(4, 2:end)))), 'b')
            plot(plot_t, 3 * sqrt(P_store(4, 2:end)), '-r')
            plot(plot_t, -3 * sqrt(P_store(4, 2:end)), '-r')
            ylabel("Translation Direction Error [rad]")
            xlabel("Time [s]")
            hold off
            legend("Initial Guess", "Estimated Error", "3-Sigma bound")
        case 4
            figure
            hold on
            title(title_names(i))
            plot(0, angle_sub(x_store(5, 1) - theta_BA_true), 'ok')
            plot(plot_t, angle_sub(x_store(5, 2:end) - theta_BA_true * ones(size(x_store(5, 2:end)))), 'b')
            plot(plot_t, 3 * sqrt(P_store(5, 2:end)), '-r')
            plot(plot_t, -3 * sqrt(P_store(5, 2:end)), '-r')
            ylabel("Rotation Error [rad]")
            xlabel("Time [s]")
            hold off
            legend("Initial Guess","Estimated Error", "3-Sigma bound")
    end
end
