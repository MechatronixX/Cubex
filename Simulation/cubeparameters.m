% -- Units --
% Weight
kg_ = 1;
g_ = 0.001*kg_;

% Size
m_ = 1;
cm_ = 0.01*m_;
mm_ = 0.1*cm_;

% Angle conversion factors 
rad_ = 1;
deg_ = pi/180;

% Time
s_ = 1;
ms_ = s_/1000;
min_ = 60*s_;
h_ = 60*min_;
d_ = 24*h_;

% -- Load reference parameters --
Linearized_system

% -- Simulation --
T_onboard = 0.002*s_;
cube_start_angle = 0.1*deg_;

% -- Geometry --
% Reference system definition:
% x-axis along motor 1
% y-axis along motor 2
% z-axis along motor 3
% Origin in common corner to all motor sides

side_length = 15*cm_;
wheel_radius = 6*cm_;

% imu_intended_alignment = [0;0;-45*deg_]; % Intended montage
% imu_misalignment = [0;0;-2.5]*deg_; % From intended montage
imu_intended_alignment = [0;0;-135*deg_]; % Intended montage
imu_misalignment = [0;0;0]*deg_; % From intended montage
angle_imu2cog = 45*deg_; % Add this to IMU angle to get cog angle compared to g

r_cube2imu = e2t([0;0;pi/4])*[55;70;10]*mm_; % Vector from origin to imu. Grovt mätt.

l_corner2cog = side_length/sqrt(2);

% -- Mass and moment of inertia --
% Beräkna kuben som sex sidor, roterandes kring en kant
% Två sidor är ortogonal (o) mot kanten, fyra sidor är parallella (p)

mass_cube = 3.487*kg_;
mass_wheel = 273*g_;   % Björn-Erik: Acc to CAD model (from Linearized_system.m)

m_s = mass_cube/6; % Massa per sida
s = side_length;   % Sidlängd

I_p = m_s*s^2/6;
I_o = m_s*s^2/12;

% I_cube: Använd Steiners sats (parallel axis theorem)
I_cube = 2*(I_p + m_s*2*(s/2)^2) ...
    + 2*(I_o + m_s*s^2/4) ...
    + 2*(I_o + m_s*((s/2)^2+s^2));
% I_wheel: Multiply by factor<1 to compensate for that the wheel does not
% have all its mass in the outer circle.
% Close to I_wheel = 8*10^-4 Björn-Erik: Acc to CAD model (from Linearized_system.m)
I_wheel = mass_wheel*(wheel_radius*0.9)^2;


% -- Sensors --
% IMU
% From ../measurements/useimudata
imu_noise_a_standard_deviation = 0.044;
imu_noise_w_standard_deviation = 0.015;
imu_bias_a_x = randn()*0.06;
imu_bias_a_y = randn()*0.06;
imu_bias_a_z = randn()*0.06;
imu_bias_w_x = randn()*0.02;
imu_bias_w_y = randn()*0.02;
imu_bias_w_z = randn()*0.02;
imu_scale_factor_a_x = 1+randn()*0.002;
imu_scale_factor_a_y = 1+randn()*0.002;
imu_scale_factor_a_z = 1+randn()*0.002;
imu_scale_factor_w_x = 1+randn()*0.002;
imu_scale_factor_w_y = 1+randn()*0.002;
imu_scale_factor_w_z = 1+randn()*0.002;
sensor_seed = 123123;

% See data sheet
a_max = 2*g;
a_scaling = 16384;
w_max = 250*deg_/s_;
w_scaling = 131;

% Error estimation
calibration_length = 2*s_; % Per side
n_samples = floor(calibration_length/T_onboard);

% Wheel sensor
wheel_noise_standard_deviation = 0.01;
wheel_sensor_seed = 666;

% -- Complementary filter
alfa = 0.995;

% -- Other properties --
Km_alt = 5.115537e-3;   % Taken from Erik and Björns simulation of motor (EdgeBalace_LQR.slx/Motor 70W brushless) (which differs from value in Linearized_system.m)
F_cube = 0.0001;        % From Linearized_system.m
F_wheel = 0.05*10^-3;   % From Linearized_system.m
