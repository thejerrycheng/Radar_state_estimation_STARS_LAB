%% Radar 2 Radar Extrinsic Cal
close all
clear
%Pre data gen init
R_A = 0.01 * diag([1, 1]);
R_B = 0.01 * diag([1, 1]);
R = [R_A, zeros(2); zeros(2), R_B];
data = gen_data();

%Post data gen init
Q = construct_F(data.dt) * diag([0.01, 1, 1, 0.025, 0.0001, 0.000000001]) * transpose(construct_F(data.dt));
t_A_BA = theta2dcm(data.theta_AV) * (data.t_V_BV - data.t_V_AV);
theta_t_true = angle_sub(atan2(t_A_BA(2,1), t_A_BA(1,1)));
theta_BA_true= angle_sub(data.theta_BV - data.theta_AV);
P = diag([0.01, 0.01, 1, 1, 10/180*pi, 10/180*pi]);
x = [data.A_meas(1:2, 1) + sqrt(P(1,1)) * randn(2,1); data.rot_vel_vi(1) * norm(t_A_BA) + sqrt(P(3,3)) * randn(1); diff(data.rot_vel_vi(1:2))* norm(t_A_BA) + sqrt(P(4,4)) * randn(1); theta_t_true + sqrt(P(5,5)) * randn(1); theta_BA_true + sqrt(P(6,6)) * randn(1)];
%x = [data.A_meas(1:2, 1); data.rot_vel_vi(1) * norm(t_A_BA); diff(data.rot_vel_vi(1:2))* norm(t_A_BA); theta_t_true; theta_BA_true];
%x = ones(6, 1);
x_store = zeros(6, length(data.t));
x_store(:, 1) = x;
P_store = zeros(6, length(data.t));
P_store(:, 1) = diag(P);
meas = [data.A_meas; data.B_meas];

F = construct_F(data.dt);
%EKF
for i=2:length(data.t)
    x_pred = f(x, data.dt);
    x_pred(5, 1) = angle_sub(x_pred(5, 1));
    x_pred(6, 1) = angle_sub(x_pred(6, 1));
    P_pred = F*P*F'  + Q;
    G = construct_G(x_pred);
    S = G * P_pred * G' + R;
    K = (P_pred * G') / S;
    P = (eye(6) - K * construct_G(x_pred)) * P_pred;
    P_store(:, i) = diag(P);
    x = x_pred + K * (meas(:, i) - g(x_pred));
    meas_error(:, i) = meas(:, i) - g(x_pred);
    x(5, 1) = angle_sub(x(5, 1));
    x(6, 1) = angle_sub(x(6, 1));
    x_store(:, i) = x;
end

title_names = ["Ego Velocity", "Scaled Rotational Velocity", "Scaled Acceleration", "Translation Direction", "Rotation"];
for i=1:5
    switch i
        case 1
            figure
            subplot(2, 1, 1)
            hold on
            plot(data.t, x_store(i, :) - data.A_meas(1, :), 'b')
            plot(data.t, 3 * sqrt(P_store(i, :)), '-r')
            plot(data.t, -3 * sqrt(P_store(i, :)), '-r')
            title(title_names(i))
            ylabel("X-Velocity Error [m/s]")
            hold off
            subplot(2, 1, 2)
            hold on
            plot(data.t, x_store(i+1, :) - data.A_meas(2, :) , 'b')
            plot(data.t, 3 * sqrt(P_store(i+1, :)), '-r')
            plot(data.t, -3 * sqrt(P_store(i+1, :)), '-r')
            ylabel("Y-Velocity Error [m/s]")
            xlabel("Time [s]")
            hold off
        case 2
            figure
            hold on
            title(title_names(i))
            plot(data.t, x_store(3, :) - data.rot_vel_vi * norm(t_A_BA), 'b')
            plot(data.t, 3 * sqrt(P_store(3, :)), '-r')
            plot(data.t, -3 * sqrt(P_store(3, :)), '-r')
            ylabel("Scaled Rotational Velocity Error [rad/s]")
            xlabel("Time [s]")
            hold off
        case 3
            figure
            hold on
            title(title_names(i))
            plot(data.t(1:end-1), x_store(4, 1:end-1) - diff(data.rot_vel_vi) * norm(t_A_BA), 'b')
            plot(data.t, 3 * sqrt(P_store(4, :)), '-r')
            plot(data.t, -3 * sqrt(P_store(4, :)), '-r')
            ylabel("Approximate Scaled Rotational Acceleration Error [rad/s^2]")
            xlabel("Time [s]")
            hold off
        case 4
            figure
            hold on
            title(title_names(i))
            plot(data.t, x_store(5, :) - theta_t_true * ones(size(x_store(5, :))), 'b')
            plot(data.t, 3 * sqrt(P_store(5, :)), '-r')
            plot(data.t, -3 * sqrt(P_store(5, :)), '-r')
            ylabel("Translation Direction Error [rad]")
            xlabel("Time [s]")
            hold off
        case 5
            figure
            hold on
            title(title_names(i))
            plot(data.t, x_store(6, :) - theta_BA_true * ones(size(x_store(6, :))), 'b')
            plot(data.t, 3 * sqrt(P_store(6, :)), '-r')
            plot(data.t, -3 * sqrt(P_store(6, :)), '-r')
            ylabel("Rotation Error [rad]")
            xlabel("Time [s]")
            hold off
    end
end