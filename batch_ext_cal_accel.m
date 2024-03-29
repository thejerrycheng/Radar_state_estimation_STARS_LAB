%% Radar 2 Radar Extrinsic Cal
close all
clear

%% Controls
window_length = 5;
initialization_type = 2; %gt, noisy, all ones

%% Pre Data Generation Initialization
meas_sigma = 0.1;
R_A = meas_sigma^2 * diag(ones(2 * window_length, 1));
R_B = meas_sigma^2 * diag(ones(2 * window_length, 1));
data = gen_data_2();

%% Post Data Generation Initialization

%Initial State
%t_A_BA = theta2dcm(data.theta_AV) * (data.t_V_BV - data.t_V_AV);
theta_t_true = angle_sub(atan2(data.t_A_BA(2,1), data.t_A_BA(1,1)));
%theta_BA_true= angle_sub(data.theta_BV - data.theta_AV);
theta_BA_true= data.theta_BA;
std_dev_prior = [10 * meas_sigma, 10 * meas_sigma, 10, 10, 10, 10, 10/180*pi, 10/180*pi];
std_dev_prior = std_dev_prior.^2;
std_dev_mm = [1, 1, 1, 0.0225, 0.1617, 0.117];
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
times = data.t;
t_window = times(1:window_length);
x = [reshape([noisy_poses(1:2, 1:window_length)+ meas_sigma * randn([2, window_length]); rand(1, window_length)], [], 1);...
    theta_t_true + 10 * sign(rand-0.5)/180*pi;...
    theta_BA_true + 10 * sign(rand - 0.5)/180*pi];

x = [reshape([noisy_poses(1:2, 1:window_length); data.rot_vel_vi(1:window_length) * norm(data.t_A_BA)], [], 1);...
    theta_t_true;...
    theta_BA_true];
x = add_accel_to_state(x, t_window);
init_state = [x(1:6); x(end-1:end)];

%Storage for estimated states
x_store = zeros(8, length(data.t) - window_length + 1);
x_store(:, 1) = [x(1:6, 1); x(end-1:end, 1)];
P_store = zeros(8, length(data.t) - window_length + 1);

%Define current measurement window
current_meas_window = noisy_poses(:, 1:window_length);

%Get errors and Jacobians
for j = 1:length(data.t) - window_length
    state_size = (length(x) - 2)/3;
    for i = 1:20
        [prior_err, J_prior, P] = batch_prior_accel(init_state, x, std_dev_prior);
        [motion_err, F, Q] = batch_f_accel(t_window, x, std_dev_mm);
        [meas_err, G, R_B] = batch_g_accel(current_meas_window, x);
        H = [J_prior; F; G];
        err = [prior_err; motion_err; meas_err];
        Q_total = blkdiag(P, Q, R_B);
        A = H' * Q_total *  H;
        b = H' * Q_total * err;
        if j > 1000
            A_schur = A(4:end, 4:end) - A(4:end, 1:3) * inv(A(1:3, 1:3)) * A(1:3, 4:end);
            b_schur = b(4:end) - A(4:end, 1:3) * inv(A(1:3, 1:3)) * b(1:3);
            del_x = A_schur\b_schur;
            x(4:end) = x(4:end) - 0.1 * del_x;
        else
            del_x = A\b;
            x = x - 0.1 * del_x;
        end
        [norm(del_x), err'* Q_total * err]
        x(end-1:end) = x(end-1: end) - (x(end-1:end) > pi) * 2 * pi;
        x(end-1:end) = x(end-1: end) + (x(end-1:end) < -pi) * 2 * pi;
    end
    
    %Expand
    x = [x(1:end - 2); noisy_poses(1:2, window_length + j); x(end - 5: end - 2); x(end - 1:end)];
    init_state = [x(1:6); x(end - 1:end)];
    current_meas_window = [current_meas_window, noisy_poses(:, window_length + j)];
    t_window = [t_window, times(window_length + j)];
    
    %Contract
    if j > 1
        x= x(7:end);
        current_meas_window = current_meas_window(:, 2:end);
        t_window = t_window(2:end);
    end
    
    %Get Covariance of first state
    P_temp = diag(inv(A));
    P = diag([P_temp(1:6, 1); P_temp(end-1:end, 1)]);
    
    % Store relevant values
    x_store(:, j + 1) = [x(1:6, 1); x(end-1:end, 1)];
    P_store(:, j + 1) = diag(P);
    
    %Update priors
    std_dev_prior = [P_temp(1:6)', P_temp(end-1:end, 1)'];
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
            plot(0, angle_sub(x_store(7, 1)- theta_t_true), 'ok')
            plot(plot_t, x_store(7, 2:end) - theta_t_true * ones(size(x_store(7, 2:end))), 'b')
            plot(plot_t, 3 * sqrt(P_store(7, 2:end)), '-r')
            plot(plot_t, -3 * sqrt(P_store(7, 2:end)), '-r')
            ylabel("Translation Direction Error [rad]")
            xlabel("Time [s]")
            hold off
            legend("Initial Guess", "Estimated Error", "3-Sigma bound")
        case 4
            figure
            hold on
            title(title_names(i))
            plot(0, angle_sub(x_store(8, 1) - theta_BA_true), 'ok')
            plot(plot_t, x_store(8, 2:end) - theta_BA_true * ones(size(x_store(8, 2:end))), 'b')
            plot(plot_t, 3 * sqrt(P_store(8, 2:end)), '-r')
            plot(plot_t, -3 * sqrt(P_store(8, 2:end)), '-r')
            ylabel("Rotation Error [rad]")
            xlabel("Time [s]")
            hold off
            legend("Initial Guess","Estimated Error", "3-Sigma bound")
    end
end