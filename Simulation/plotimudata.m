% Run Simulink model, write imu output to imu_data
T = 0.002;

% Extract data
t = imu_data.a_x.Time;
a_x_data = imu_data.a_x.Data;
a_y_data = imu_data.a_y.Data;
w_z_data = imu_data.w_z.Data;

% Convert to correct units
g_ = 9.81*m_/s_^2;
a_scale_factor = 16384; % AFS_SEL = 0, se datablad MPU9250
w_scale_factor = 131;   % FS_SEL = 0, se datablad MPU9250
a_x_data = a_x_data/a_scale_factor*g_;
a_y_data = a_y_data/a_scale_factor*g_;
w_z_data = w_z_data/w_scale_factor*deg_;

% Plot
close all

figure
plot(t,t,'x-')
title('t','interpreter','none')
grid on
xlabel('Time [s]')

figure
plot(t,a_x_data,'x-')
title('a_x_data','interpreter','none')
grid on
xlabel('Time [s]')

figure
plot(t,a_y_data,'x-')
title('a_y_data','interpreter','none')
grid on
xlabel('Time [s]')

figure
a1=subplot(2,1,1);
plot(t,w_z_data,'x-')
title('w_z_data','interpreter','none')
grid on
xlabel('Time [s]')

a2=subplot(2,1,2);
plot(t(1:end-1),diff(w_z_data),'x-')
title('diff(w_z_data)','interpreter','none')
grid on
xlabel('Time [s]')
linkaxes([a1 a2],'x')

figure
plot(t,sqrt(a_x_data.^2+a_y_data.^2))
title('Acceleration vector length')
grid on
xlabel('Time [s]')

%% -- Accelerometer --
%% Sections
cube_angle_1 = -45*deg_;
cube_angle_2 =  45*deg_;
i_1 = find(t < 2);
i_2 = find(t > 3 & t < 5);
% i_turn = find(t > 10.5 & t < 14.5);
a_x_1_meas = mean(a_x_data(i_1));
a_x_2_meas = mean(a_x_data(i_2));
a_y_1_meas = mean(a_y_data(i_1));
a_y_2_meas = mean(a_y_data(i_2));

%% Find misalignment
g = 9.81;
scale_factor = 1; % Vi vet inte bättre än så

% Utgår ifrån att dessa sektioner borde visa samma värde. Felet kan bara
% bero på misalignment i så fall.

misalignment_x = asin(1/(sqrt(2)*scale_factor*g)*(a_x_1_meas+a_x_2_meas));
misalignment_y = asin(1/(sqrt(2)*scale_factor*g)*(a_y_1_meas-a_y_2_meas));

misalignment = mean([misalignment_x,misalignment_y]);
% misalignment = misalignment_x;
% misalignment = imu_misalignment(3); % TEST

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
    scale_factor_a_x = (a_x_1_meas-a_x_2_meas)/(a_x_1_true-a_x_2_true)
else
    scale_factor_a_x = 1
end
if abs(a_y_1_true - a_y_2_true) > min_diff_s
    scale_factor_a_y = (a_y_1_meas-a_y_2_meas)/(a_y_1_true-a_y_2_true)
else
    scale_factor_a_y = 1
end
bias_a_x = (a_x_2_true*a_x_1_meas-a_x_1_true*a_x_2_meas)/(a_x_2_true-a_x_1_true)
bias_a_y = (a_y_2_true*a_y_1_meas-a_y_1_true*a_y_2_meas)/(a_y_2_true-a_y_1_true)


%% Calculate cube angle from accelerometer
cube_angle_meas_a = atan2((a_x_data-bias_a_x)/scale_factor_a_x,(a_y_data-bias_a_y)/scale_factor_a_y)-misalignment;

figure
plot(t,cube_angle_meas_a/deg_)
grid on

%% -- Gyroscope --
%% Bias
bias_w_z = mean([w_z_data(i_1);w_z_data(i_2)]);

%% Scale factor
% Due to random walk, I don't dare say anything about the scale factor
% Knowing the standard deviation of the noise, the probability of the
% measured scale factor not being a random walk could be calculated.

% scale_factor_w_z = sum(w_z_data(i_turn))*T/(pi/2);
scale_factor_w_z = 1;

%% Calculate cube angle from gyroscope
cube_angle_meas_w = (cumsum(w_z_data)-bias_w_z)*T+cube_angle_meas_a(1);

%% -- Complementary filter --
% theta_k = alfa*(theta_k_1 + w_k*delta_t) + (1-alfa)*a_k
alfa = 0.95;
theta = zeros(size(t));

theta(1) = atan2((a_x_data(1)-bias_a_x)/scale_factor_a_x,(a_y_data(1)-bias_a_y)/scale_factor_a_y)-misalignment;
k_0 = find(t > 5);
for k = k_0:length(theta)
    a_k = atan2((a_x_data(k)-bias_a_x)/scale_factor_a_x,(a_y_data(k)-bias_a_y)/scale_factor_a_y)-misalignment;
    w_k = w_z_data(k) - bias_w_z;
    theta(k) = alfa*(theta(k-1) + w_k*T) + (1-alfa)*a_k;
end

figure
hold all
plot(t,cube_angle_meas_a/deg_)
plot(t,cube_angle_meas_w/deg_)
plot(t,theta/deg_)
grid on
legend('From acc','From gyro','Complementary filter')

%% Test different alfa

alfa = [0.95 0.98 0.99 0.995 0.999];
theta = zeros(length(t),length(alfa));

theta_0 = atan2((a_x_data(1)-bias_a_x)/scale_factor_a_x,(a_y_data(1)-bias_a_y)/scale_factor_a_y)-misalignment;

theta(1,:) = theta_0;
k_0 = find(t > 5);
for a = 1:length(alfa)
    for k = k_0:length(theta)
        a_k = atan2((a_x_data(k)-bias_a_x)/scale_factor_a_x,(a_y_data(k)-bias_a_y)/scale_factor_a_y)-misalignment;
        w_k = w_z_data(k) - bias_w_z;
        theta(k,a) = alfa(a)*(theta(k-1,a) + w_k*T) + (1-alfa(a))*a_k;
    end
end

alfa_legend = {};
figure
a1=subplot(3,1,1);
hold all
for a = 1:length(alfa)
    plot(t,theta(:,a)/deg_)
    alfa_legend = [alfa_legend num2str(alfa(a))];
end
grid on
legend(alfa_legend)
a2=subplot(3,1,2);
plot(t,cube_angle_meas_a/deg_)
grid on
a3=subplot(3,1,3);
plot(t,cube_angle_meas_w/deg_)
grid on
linkaxes([a1 a2 a3],'x')

% alfa = 0.995 verkar vettigt

%% -- Measurement error (for simulation) --
e_a_x = mean([std(a_x_data(i_1)) std(a_x_data(i_2))]);
e_a_y = mean([std(a_y_data(i_1)) std(a_y_data(i_2))]);
e_w_z = mean([std(w_z_data(i_1)) std(w_z_data(i_2))]);
