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


skew_matrix =@(P) [ 0     -P(3)    P(2);
                    P(3)     0     -P(1);
                    -P(2)    P(1)      0];


 %function S = skew_matrix(P)
% %   argument P most be a position vector in three dimenstion
% %   return a skew_matrix
% assert(size(P,1) == 3,'Argurment P most be of size 3x1')
% 
% S = [ 0     -P(3)    P(2);
%      P(3)     0     -P(1);
%     -P(2)    P(1)      0];
% end


%% Wheel 
% Is what already is stated above, but in a struct 
%All defined in the wheels principal frame NOT in the cube frame, nor the
%global frame 

wheel = struct( 'm',        273*g_,...     % Wheel mass  % Björn-Erik: Acc to CAD model (from Linearized_system.m)
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
                'J',  [],...   %Inertia around shaft 
                'bq', 0,...         %Quadratic damping Tq = bc*w^2 TODO: System identification
                'bl', 0 );           %Linear damping    Tl = bl*w;          
      
% I_wheel: Multiply by factor<1 to compensate for that the wheel does not
% have all its mass in the outer circle.
wheel.J     = wheel.m*(wheel.radius*0.9)^2;
wheel.Iw0    = (1/12)*wheel.m*(3*wheel.radius^2 + wheel.h^2); 


%---------Wheel damping
% TODO: Employ system identification to get this better
%Set quadratic damping coeffcieint such that 1 Nm of torque input gives 
%equilibrium at 10 000 RPM ~ 1000 Rad/s. The quadratic term was ditched in
%favor or a simpler linear damping model, look into this 

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
               'rcb',          [],...   %Vector from corner to center of gravity of the cube casing  
               'I_tilde_1',    [] );
           
%Seems that struct members must be initalized like this when they depend on
%one another
% cube.m_tot             = 3.487*kg_;                   %Complete cube mass, wheels batteries and all  
cube.m_tot             = 2900*g_;                       %Complete cube mass, wheels batteries and all 
cube.l                 = 180*mm_;                       %Length of one side
cube.r                 = cube.l/2; 
cube.rcb               = [cube.r; cube.r ; cube.r];     %Vector from cube corner -> COG
cube.Ix                = 1/6*cube.m_tot*cube.l^2;       %Principal inertia for cuboid w. evenly distrib mass
cube.Iy                = cube.Ix; 
cube.Iz                = cube.Ix;
cube.l_corner2cog      = sqrt(2)*cube.r;                              %From a corner to the centerpoint in the plane of a side
cube.I_2D              = cube.Iy+cube.m_tot*cube.l_corner2cog^2;      %Convenience inertia when edge balancing on an edge
cube.Ic                = cube.Ix; 
cube.tensor            = diag([cube.Ic,cube.Ic,cube.Ic]); 
cube.I_tilde_1         = -cube.m_tot.*skew_matrix(cube.rcb)^2;


                     
                     
% The position vectors from the corner to the center of gravity of the
% wheels is a function of the side
wheel.rw1               = [0 ; cube.r ; cube.r];
wheel.rw2               = [cube.r ; 0 ; cube.r];
wheel.rw3               = [cube.r ; cube.r ; 0];
wheel.rw                = wheel.rw1 + wheel.rw2 + wheel.rw3;

wheel.I_tilde_2         = -wheel.m.*skew_matrix(wheel.rw1)^2;
wheel.I_tilde_3         = -wheel.m.*skew_matrix(wheel.rw2)^2;
wheel.I_tilde_4         = -wheel.m.*skew_matrix(wheel.rw3)^2;  

%The wheel spins around its z-axis in its own coordinate system 
wheel.Iz = wheel.J;  
wheel.Theta_w0 = diag(wheel.Iw0*ones(3,1));
wheel.Theta_z  = diag(wheel.Iz*ones(3,1));

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
         
imu = struct('a_max', single(2*9.81),...                    % Saturation for the current settings in m/s^2
             'a_scaling', single(16384),...                 % LSB/g
             'd2c',       single(20*mm_),...                %Distance from IMU to corner of cube
             'w_max', single(250*deg_/s_),...               % Rad
             'w_scaling', single(131),...                   % LSB/(º/s)
             'm_max', single(4800),...                      % microTesla 
             'm_scaling',single(0.6),...                    % microTesla/LSB
             'rot_IMU1', single([0.6024 0.7982 -0.0045;
                         -0.7982 0.6024  0.0049;
                          0.0066 0.0006  1.0000]),...% Rotation matrix IMU1
             'rot_IMU3', single([-0.6330 0.7739  0.0199;
                          -0.7731  -0.6333 0.0359
                          0.0404  0.0074  0.9992])); % Rotation matrix IMU3      
             
