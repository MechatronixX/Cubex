% -- Units --

% Forces 
mNm_ = 0.001; 

%Electrical 
V_RPM_ = 30/pi; %Converts from voltage/RPM --> voltage/rad 
V_     = 1;     %Voltage 
% Weight
kg_ = 1;
g_ = 0.001*kg_;

% Lengths
m_  = 1;
cm_ = 0.01;
mm_ = 0.001;

% Angle conversion factors 
rad_ = 1;
deg_ = pi/180;
rpm_ = 60/(2*pi);

% Time
s_      = 1;
ms_     = s_/1000;
min_    = 60*s_;
h_      = 60*min_;
d_      = 24*h_;

% -- Load reference parameters --
Linearized_system; %TODO: Whats this?? 
clearvars A_I A Qx Qu Q
% -- Simulation --
T_onboard           = 0.002*s_;
cube_start_angle    = 0.1*deg_;

% -- Geometry --
% Reference system definition:
% x-axis along motor 1
% y-axis along motor 2
% z-axis along motor 3
% Origin in common corner to all motor sides

% -- Mass and moment of inertia --
% Ber�kna kuben som sex sidor, roterandes kring en kant
% Tv� sidor �r ortogonal (o) mot kanten, fyra sidor �r parallella (p)

% m_s = mass_cube/6; % Massa per sida
% s = side_length;   % Sidl�ngd
% 
% I_p = m_s*s^2/6; %So cube inertia is approximated as a straight bar? 
% I_o = m_s*s^2/12;
% 
% % I_cube: Anv�nd Steiners sats (parallel axis theorem)
% I_cube = 2*(I_p + m_s*2*(s/2)^2) ...
%     + 2*(I_o + m_s*s^2/4) ...
%     + 2*(I_o + m_s*((s/2)^2+s^2));

%% Wheel 
% Is what already is stated above, but in a struct 
%All defined in the wheels principal frame NOT in the cube frame, nor the
%global frame 

wheel = struct( 'm',        273*g_,...     % Wheel mass  % Bj�rn-Erik: Acc to CAD model (from Linearized_system.m)
                'radius',   60*mm_,...     % The radius of the wheel
                'h',  6*mm_,...            % Thickness of the reaction wheel
                'l', 90*mm_,...            % Length from corner to wheel center
                'Iw0',[],...
                'Iz', [],... %Todo: Rename as Iwz to comply with report
                'I_tilde_2', [],...     %Added inertia due to displacment, se report 
                'I_tilde_3', [],...
                'I_tilde_4', [],...
                'rw1',[],...   % Position vector from corner to wheel 1 defined in body frame
                'rw2',[],...   % Position vector from corner to wheel 2 defined in body frame
                'rw3',[],...   % Position vector from croner to wheel 3 defined in body frame
                'rw', [],...  
                'Theta_w0', 0,...
                'Theta_z', 0,...
                'J',  I_wheel,...   %Inertia around shaft 
                'bq', 0,...         %Quadratic damping Tq = bc*w^2 TODO: System identification
                'bl', 0 );           %Linear damping    Tl = bl*w;          
      
% I_wheel: Multiply by factor<1 to compensate for that the wheel does not
% have all its mass in the outer circle.
% Close to I_wheel = 8*10^-4 Bj�rn-Erik: Acc to CAD model (from Linearized_system.m)
wheel.J     = wheel.m*(wheel.radius*0.9)^2;
wheel.Iw0    = (1/12)*wheel.m*(3*wheel.radius^2 + wheel.h^2); 

%The wheel spins around its z-axis in its own coordinate system 
wheel.Iz = wheel.J;  
wheel.Theta_w0 = diag(wheel.Iw0*ones(3,1));
wheel.Theta_z  = diag(wheel.Iz*ones(3,1));

%TODO: These vectors have the right structure but  should be a function of
%cube.r 
wheel.rw1               = [0 ; wheel.l ; wheel.l];
wheel.rw2               = [wheel.l ; 0 ; wheel.l];
wheel.rw3               = [wheel.l ; wheel.l ; 0];
wheel.rw                = wheel.rw1 + wheel.rw2 + wheel.rw3;

wheel.I_tilde_2         = -wheel.m.*skew_matrix(wheel.rw1)^2;
wheel.I_tilde_3         = -wheel.m.*skew_matrix(wheel.rw2)^2;
wheel.I_tilde_4         = -wheel.m.*skew_matrix(wheel.rw3)^2;


%---------Wheel damping
% TODO: Employ system identification to get this better
%Set quadratic damping coeffcieint such that 1 Nm of torque input gives 
%equilibrium at 10 000 RPM ~ 1000 Rad/s 

%wheel.b = 1/1000^2; 
wheel.bq = 60/1000^2;
wheel.bl = 70/1000^2; 
            
%% Cube

%TODO: Should there rather be one struct called casing, and then another
%called cube handling the whole cube? 
cube = struct( 'm_tot',        [] ,...  
               'r',            [],...
               'l',            [],...                      
               'l_corner2cog', [],... 
               'alpha_YZ',     0*deg_,... %Nominal angle between cube bottom and the COG in the YZ plane TODO: Add to model properly
               'Ix',           [],...        
               'Iy',           [],...                 
               'Iz',           [],...
               'I_2D',         [],...
               'Ic',           [],...    %Principle moments of inertia
               'tensor',       [],...
               'I3D',          [],...   %3D equivalent inertia tensor, when all mass displacement, wheels etc are included
               'I3D_tilde',    [],...     
               'rcb',          [],...   %Vector from corner to center of gravity  
               'I_tilde_1',    [] );
           
%Seems that struct members must be initalized like this when they depend on
%one another
% cube.m_tot             = 3.487*kg_;                    %Complete cube mass, wheels batteries and all  
cube.m_tot             = 2900*g_;                    %Complete cube mass, wheels batteries and all 
cube.l                 = 180*mm_;                      %Length of one side
cube.r                 = cube.l/2; 
cube.rcb               = [cube.r; cube.r ; cube.r]; %Vector from cube corner -> COG
cube.Ix                = 1/6*cube.m_tot*cube.l^2;      %Principal inertia for cuboid w. evenly distrib mass
cube.Iy                = cube.Ix; 
cube.Iz                = cube.Ix;
cube.l_corner2cog      = sqrt(2)*cube.l*0.5;               %From a corner to the centerpoint
cube.I_2D              = cube.Iy+cube.m_tot*cube.l_corner2cog^2;      %Convenience inertia when edge balancing on an edge
cube.Ic                = cube.Ix; 
cube.tensor            = diag([cube.Ic,cube.Ic,cube.Ic]); 
cube.I_tilde_1         = -cube.m_tot.*skew_matrix(cube.rcb)^2;
cube.I3D               = cube.I_tilde_1 + wheel.I_tilde_2 + wheel.I_tilde_3 +...
                         wheel.I_tilde_4 + cube.tensor + wheel.Theta_w0;
cube.I3D_tilde         = cube.I_tilde_1 + wheel.I_tilde_2 + wheel.I_tilde_3 +...
                         wheel.I_tilde_4;

%% Motors 
%EC45 motor datasheet http://www.maxonmotor.com/maxon/view/news/MEDIENMITTEILUNG-EC-45-flat-70-Watt

%TODO: L and R here are phase to phase, not eq. DC motor! 

motor = struct('L', 0.00463,...             % Equivalent DC motor inductance
               'R', 0.6,...                 % Equivalent DC motor resistance
               'kw', 1/259*V_RPM_ ,...      % Motor voltage
               'kt', 36.9*mNm_,...          % Motor torque constant
               'tau_cl', 0.03,...           % Time constant closed loop current controller
               'Imax', 4,...                % Max permissible motor current
               'Vbat', 24*V_,...            % Nominal batter voltage
               'tau',  []);                 % Electrial time constant

%motor.kt = motor.kt *5;  %DEBUG!!!!
motor.tau = motor.L/motor.R; 


%% Motor Controllers Maxon 

% Product Specification:            http://www.maxonmotor.com/medias/sys_master/root/8818447941662/414533-ESCON-36-3-EC-Hardware-Reference-En.pdf

controller = struct('Amax' , 4,...          % Max output current
                    'PWM_max', 90,...       % PWM max duty cycle
                    'PWM_min', 10);         % PWM min duty cycle

%% Sampling intervals 
 Ts = struct('controller',  0.016,...
             'IMU',         0.008,...
             'base',        0.002); 
         
%% MPC
[MPC , fMPC]     =    MPC_Parameters(cube, motor, Ts);

%% Sensor
% IMU
% Register Map and Descriptions:    https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU-9250-Register-Map.pdf
% Product Specification:            https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU9250REV1.0.pdf

% imu_intended_alignment = [0;0;-45*deg_];  % Intended montage
% imu_misalignment = [0;0;-2.5]*deg_;       % From intended montage
imu_intended_alignment = [0;0;-135*deg_];   % Intended montage
imu_misalignment = [0;0;0]*deg_;            % From intended montage
angle_imu2cog = 45*deg_;                    % Add this to IMU angle to get cog angle compared to g

r_cube2imu = e2t([0;0;pi/4])*[55;70;10]*mm_; % Vector from origin to imu. Grovt m�tt.


imu = struct('a_max', single(2*g),...                       % m/s^2
             'a_scaling', single(16384),...                 % LSB/g
             'd2c',       single(20*mm_),...                %Distance from IMU to corner of cube
             'w_max', single(250*deg_/s_),...               % Rad
             'w_scaling', single(131),...                   % LSB/(�/s)
             'm_max', single(4800),...                      % microTesla 
             'm_scaling',single(0.6),...                    % microTesla/LSB
             'rot_IMU1', single([0.6024 0.7982 -0.0045;
                         -0.7982 0.6024  0.0049;
                          0.0066 0.0006  1.0000]),...% Rotation matrix IMU1
             'rot_IMU3', single([-0.6330 0.7739  0.0199;
                          -0.7731  -0.6333 0.0359
                          0.0404  0.0074  0.9992])); % Rotation matrix IMU3      
             
            
%TOD0:Remove? 
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
Km_alt = 5.115537e-3;   % Taken from Erik and Bj�rns simulation of motor (EdgeBalace_LQR.slx/Motor 70W brushless) (which differs from value in Linearized_system.m)
F_cube = 0.0001;        % From Linearized_system.m
F_wheel = 0.05*10^-3;   % From Linearized_system.m

%I cannot get the script to run while this stuff is in here
function S = skew_matrix(P)
%   argument P most be a position vector in three dimenstion
%   return a skew_matrix
assert(size(P,1) == 3,'Argurment P most be of size 3x1')

S = [ 0     -P(3)    P(2);
     P(3)     0     -P(1);
    -P(2)    P(1)      0];
end