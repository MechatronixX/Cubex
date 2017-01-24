function [misalignment,bias_a_x,bias_a_y,bias_w_z,scale_factor_a_x,scale_factor_a_y,scale_factor_w_z] = CalculateImuErrors2D(a_x_1_meas,a_x_2_meas,a_y_1_meas,a_y_2_meas,w_z_meas)
%#codegen
% Assumes cube_angle_1 = -45 deg, cube_angle_2 = 45 deg

%% Find misalignment
cube_angle_1 = -45*deg_;
cube_angle_2 =  45*deg_;
g = 9.81;
scale_factor = 1; % Vi vet inte bättre än så

% Utgår ifrån att dessa sektioner borde visa samma värde (x med omvänt tecken).
% Felet kan bara bero på misalignment i så fall.
% Equations based on trigonometry
misalignment_x = asin(1/(sqrt(2)*scale_factor*g)*(a_x_1_meas+a_x_2_meas));
misalignment_y = asin(1/(sqrt(2)*scale_factor*g)*(a_y_1_meas-a_y_2_meas));

misalignment = mean([misalignment_x,misalignment_y]);
% misalignment = misalignment_x;
% misalignment = imu_misalignment(3); % TEST
% misalignment = -0.043633231299858;
%% Find bias and scale factor
% Derive expressions from this equations system:
%  / a_1_measured = scale_factor*a_2_true + bias
% {
%  \ a_2_measured = scale_factor*a_2_true + bias

% What we should measure. Account for misalignment
a_x_1_true = -g*sin(-cube_angle_1-misalignment);
a_y_1_true =  g*cos(-cube_angle_1-misalignment);

a_x_2_true =  g*sin( cube_angle_2+misalignment);
a_y_2_true =  g*cos( cube_angle_2+misalignment);

% Derived expressions
min_diff_s = 1e-3; % Godtyckligt vald
if abs(a_x_1_true - a_x_2_true) > min_diff_s % To avoid numerical problems
    scale_factor_a_x = (a_x_1_meas-a_x_2_meas)/(a_x_1_true-a_x_2_true);
else
    scale_factor_a_x = 1;
end
if abs(a_y_1_true - a_y_2_true) > min_diff_s
    scale_factor_a_y = (a_y_1_meas-a_y_2_meas)/(a_y_1_true-a_y_2_true);
else
    scale_factor_a_y = 1;
end
bias_a_x = (a_x_2_true*a_x_1_meas-a_x_1_true*a_x_2_meas)/(a_x_2_true-a_x_1_true);
bias_a_y = (a_y_2_true*a_y_1_meas-a_y_1_true*a_y_2_meas)/(a_y_2_true-a_y_1_true);

%% -- Gyroscope --
%% Bias
bias_w_z = w_z_meas;

%% Scale factor
% Due to random walk, I don't dare say anything about the scale factor
% Knowing the standard deviation of the noise, the probability of the
% measured scale factor not being a random walk could be calculated.

% scale_factor_w_z = sum(w_z_data(i_turn))*T/(pi/2);
scale_factor_w_z = 1;