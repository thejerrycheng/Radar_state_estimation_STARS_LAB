function [C3_out ,C5_out, vel_x3_out, vel_y3_out, vel_x5_out, vel_y5_out] = interpretation_solver(C3, C5, vel_x3, vel_y3, vel_x5, vel_y5)

Fx3 = griddedInterpolant(vel_x3);
Fx5 = griddedInterpolant(vel_x5);
Fy3 = griddedInterpolant(vel_y3);
Fy5 = griddedInterpolant(vel_y5);

for i = 1: size(C3,2)
C3_reshape{i} = reshape(C3{i}, [4,1]);
end 
C3_reshape_mat = cell2mat(C3_reshape);

for i = 1: size(C5,2)
C5_reshape{i} = reshape(C5{i}, [4,1]);
end 
C5_reshape_mat = cell2mat(C5_reshape);

Funcertainty_1_3 = griddedInterpolant(C3_reshape_mat(1,:));
Funcertainty_1_5 = griddedInterpolant(C5_reshape_mat(1,:));
Funcertainty_2_3 = griddedInterpolant(C3_reshape_mat(2,:));
Funcertainty_2_5 = griddedInterpolant(C5_reshape_mat(2,:));
Funcertainty_3_3 = griddedInterpolant(C3_reshape_mat(3,:));
Funcertainty_3_5 = griddedInterpolant(C5_reshape_mat(3,:));
Funcertainty_4_3 = griddedInterpolant(C3_reshape_mat(4,:));
Funcertainty_4_5 = griddedInterpolant(C5_reshape_mat(4,:));

out_x = [Fx3(linspace(1,numel(vel_x3),8000)'), Fx5(linspace(1,numel(vel_x5),8000)')];
out_y = [Fy3(linspace(1,numel(vel_y3),8000)'), Fy5(linspace(1,numel(vel_y5),8000)')];

out_uncertainty_1 = [Funcertainty_1_3(linspace(1,numel(C3_reshape_mat(1,:)),8000)'), Funcertainty_1_5(linspace(1,numel(C5_reshape_mat(2,:)),8000)')];
out_uncertainty_2 = [Funcertainty_2_3(linspace(1,numel(C3_reshape_mat(2,:)),8000)'), Funcertainty_2_5(linspace(1,numel(C5_reshape_mat(2,:)),8000)')];
out_uncertainty_3 = [Funcertainty_3_3(linspace(1,numel(C3_reshape_mat(1,:)),8000)'), Funcertainty_3_5(linspace(1,numel(C5_reshape_mat(2,:)),8000)')];
out_uncertainty_4 = [Funcertainty_4_3(linspace(1,numel(C3_reshape_mat(2,:)),8000)'), Funcertainty_4_5(linspace(1,numel(C5_reshape_mat(2,:)),8000)')];

vel_x3_out = out_y(:,1);
vel_x5_out = out_y(:,2);

vel_y3_out = out_y(:,1);
vel_y5_out = out_y(:,2);

uncertainty_1_3_out = out_uncertainty_1(:,1);
uncertainty_1_5_out = out_uncertainty_1(:,2);
uncertainty_2_3_out = out_uncertainty_2(:,1);
uncertainty_2_5_out = out_uncertainty_2(:,2);
uncertainty_3_3_out = out_uncertainty_3(:,1);
uncertainty_3_5_out = out_uncertainty_3(:,2);
uncertainty_4_3_out = out_uncertainty_4(:,1);
uncertainty_4_5_out = out_uncertainty_4(:,2);

C3_out = [uncertainty_1_3_out,uncertainty_2_3_out,uncertainty_3_3_out,uncertainty_4_3_out]';
C5_out = [uncertainty_1_5_out,uncertainty_2_5_out,uncertainty_3_5_out,uncertainty_4_5_out]';

end 