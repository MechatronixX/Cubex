%% Calculating LQR feedback parameters for the linearized 2d edge balancing problem
%For simulation, use "run_simulation_2D" 

%clear all; 

%Load the cube parameters 
cubeparameters; 


%Rename for  readability 
m_tot   = cube.m_tot; 
l       = cube.l_corner2cog; 
I2D     = cube.I_2D; 
kt      = motor.kt; %DEBUG: Seems that current become too large
%% Continous system matrices 


% %------------Model with current reference as insignal 
% 
% tau_i = motor.tau_cl; % Time constant i_ref --> i
% 
% A = [0                                                  1                               0 ;
%     cube.m_tot*cube.l_corner2cog/cube.I_2D*g            0                               motor.kt/cube.I_2D;
%     0                                                   0                               -1/tau_i];  
% B = [0; 0; 1/tau_i]; 
% 
% C = [1 0 0]; 
% 
% D =[];
% 
% inputnames ='i_ref'; 
% statenames = {'theta_c'  'omega_c' 'i' };




% %------------Model with moment as insignal 
% 
%tau_i = motor.tau_cl; % Time constant i_ref --> i

A = [0                        1;                              
    m_tot*l*g/I2D             0;]                              
    
B = [0; kt/I2D]; 

C = [1 0; 
     0 1]; 

D =[];

inputnames ='torque'; 
statenames = {'theta_c'  'omega_c'};

% %--------Model with voltage as insignal 
% A = [0                                          1                           0                        0;
%     cube.m_tot*cube.l_corner2cog/cube.I_2D*g    0                           0                        motor.kt/cube.I_2D;
%     0                                           0                           0                       -motor.kt/wheel.Iy;
%     0                                           -motor.kw/motor.L           motor.kw/motor.L        -motor.R/motor.L];
% 
% B = [0 ; 0; 0;  1/motor.L]; 
% 
% C = [1 0 0 0]; 
% % C  = [1 0 0 0; 
% %       0 1 0 0;
% %       0 0 1 0;
% %       0 0 0 0];
%   
% D=0; 
% inputnames ='Vin'; 
% statenames = {'theta_c'  'omega_c'  'omega_m' 'i' };

% %Model with torque as input
% A = [0                                                      1; 
%      (cube.m_tot)*(cube.l_corner2cog)*g/(cube.I_2D)         0]; 
%  
% B = [0 ; 2/cube.I_2D]; 
% 
% C =[1 0]; 
% 
% inputnames ='Torque'; 
% statenames = {'theta_c'  'omega_c' };


%-----Pack system 
sys_c = ss(A,B,C,[], 'Inputname',inputnames, 'Statename',statenames); 

Nx = length(A); 

%% System discetization
Ts = 0.02;  %Sampling time of choice 
sys_d = c2d(sys_c, Ts);

%% Reachability 

%-----Cont. time rechability
Co_cont = ctrb(sys_c); 
disp(['Continous time controllability matrix rank = ', num2str(rank(Co_cont))      ] );


%---Discrete reachability 
Co_disc = ctrb(sys_d); 

disp(['Discrete time reachability matrix rank = ', num2str(rank(Co_disc))      ] );


%% LQR 
Nx = length(A); 

if(Nx == 4)
        disp('Four state model: [Theta_c , omega_c, omega_1 , i_1 ]')
        Qx = diag([10000 0.1 0.1 0.1]);   %Penalties on states, we care mostly about the angle 
        Ru = 1;                       %Voltage is our only input
        x0= [pi/4 ; 0 ; 0;0]
elseif (Nx == 2)
        disp('Using two state model excluding motor model x = [Theta_c, omegac ]'); 
        Qx = diag([1 0.001]); 
        Ru = 1310000000; 
        x0= [deg2rad(4) ; 0];     
elseif (Nx == 3)
    disp('Three state model: [Theta_c , omega_c, i]')
    
      Qx = diag([0 0 1]);    
      Ru =1;                 %Insignal is a current reference for the motor
      
      x0= [deg2rad(6) ; 0 ;0 ];  
end

[K_lqr,~,~] = lqr(sys_d,Qx,Ru) 

%eigenvalues = abs(eig(sys_d.A-sys_d.B*K_lqr))








