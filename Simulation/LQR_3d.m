%% Calculating LQR feedback parameters for the linearized 3d edge balancing problem
%For simulation, use "run_simulation_2D" 

%clear all; 

%Load the cube parameters 
cubeparameters; 

%Rename for  readability 
M       = cube.m_tot; 
r = cube.r;
l       = cube.l_corner2cog; 
Ic      = cube.Ic; 
I3D     = cube.I3D; 
kt      = motor.kt; %DEBUG: Seems that current become too large
%% Continous system matrices 

theta0 = atan(sqrt(2))
g0 = 9.81; 

A = [0,0,(-1).*2.^(1/2).*g0.*Ic.^(-1).*M.*r.*cot(theta0),0,0,0;0,g0.* ...
  Ic.^(-1).*M.*r.*(cos(theta0)+2.^(1/2).*sin(theta0)),0,0,0,0;0,0, ...
  2.^(1/2).*g0.*Ic.^(-1).*M.*r.*csc(theta0),0,0,0];

%This a only models the highest derivatives, augment to include lower
%derivatives

A= [ zeros(3,3), eye(3)  ; A]

B = [2.^(-1/2).*Ic.^(-1).*csc(theta0),2.^(-1/2).*Ic.^(-1).*csc(theta0) ...
  ,0;2.^(-1/2).*Ic.^(-1),(-1).*2.^(-1/2).*Ic.^(-1),0;(-1).*2.^(-1/2) ...
  .*Ic.^(-1).*cot(theta0),(-1).*2.^(-1/2).*Ic.^(-1).*cot(theta0), ...
  Ic.^(-1)];

B = [zeros(3,3); B];

%We measure all
C = eye(6); 

inputnames ={'T1','T2','T3}'}; 
statenames = {'phi', 'theta', 'psi','phidot' , 'thetadot', 'psidot'};


sys_c = ss(A,B,C,[], 'Inputname',inputnames, 'Statename',statenames) 

Nx = length(A); 

%% System discetization
Ts = 0.002;  %Sampling time of choice 
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
      
Qx = eye(6);   %Penalties on states, we care mostly about the angle 
Ru = eye(3);   %Voltage is our only input

[K_lqr_3D,~,~] = lqr(sys_d,Qx,Ru) 

%eigenvalues = abs(eig(sys_d.A-sys_d.B*K_lqr))

%% Step response 

x0 = [0.1,0.1,0.1,0,0,5]

save('K_lqr_3D')


