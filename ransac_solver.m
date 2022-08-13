function [x, invalid_frames, range_x_in, range_y_in, vel_in]= ransac_solver(y,A)
% y as a N*1 vector; A as a N*2 matrix 
% output the linear equation solution using RANSAC to filter out the
% outliers; x as a 2*1 vector 

%% total number of radar captured points: 
    [total_num,col] = size(y);

%% normalization 

%% the hyperparameters: 
    P = 0.95;
    e = 0.1;
    init_num = 2; % the initial guess's number of samples selected 
    iteration = log(1-P)/log(1-e^init_num);
    sigma_v = 0.1; % m/s -- assumed arbitary number -- to be changed later 
    threshold = 0.01; 
    min_inliners = round(total_num*0.10);

%% first initial guess 
    first_batch_y = y(1:init_num, :);
    first_batch_A = A(1:init_num, :);
    Sigma_first_batch = 1/(sigma_v^2)*eye(init_num,init_num);
    x_first_batch = (first_batch_A'*Sigma_first_batch*first_batch_A)\(first_batch_A'*Sigma_first_batch*first_batch_y); 
    remain_num = total_num - init_num; 


 %% iterate the guess function 
 iteration_num = 0;
 invalid_frames = 0;
    for i = 1:1000 
        inliners_num = 0;
        indices = randsample(total_num,init_num);

        first_batch_y = y(indices, :);
        first_batch_A = A(indices, :); 
        %Check theta between vectors
        if abs(acos(first_batch_A(1, :) * first_batch_A(2, :)')) < 0.1
            continue
        end
        Sigma_first_batch = 1/(sigma_v^2)*eye(init_num,init_num);
        x_first_batch = (first_batch_A'*Sigma_first_batch*first_batch_A)\(first_batch_A'*Sigma_first_batch*first_batch_y);
        leftover_indices = setdiff(1:total_num, indices);
        distance_vector = abs(y(leftover_indices, :) - A(leftover_indices, :) * x_first_batch);
        inliners_ids = distance_vector < threshold;

        if sum(inliners_ids) > min_inliners % 190
            fit_indices = [indices;leftover_indices(inliners_ids)'];
            Sigma_first_batch = 1/(sigma_v^2)*eye(size(fit_indices,1),size(fit_indices,1));
            A_final = A(fit_indices, :);
            y_final = y(fit_indices, :);
            x_final_batch = (A_final'*Sigma_first_batch*A_final)\(A_final'* Sigma_first_batch * y_final);
            x = cast(x_final_batch,"double"); 
            range_x_in = A_final(:,1);
            range_y_in = A_final(:,2);
            vel_in = y;
            break

        else 
            x = zeros(2,1);
            invalid_frames = invalid_frames + 1;
            range_x_in = 0;
            range_y_in = 0;
            vel_in = 0;
        % do nothing 
        end 
        iteration_num = iteration_num + 1;
    end
end


