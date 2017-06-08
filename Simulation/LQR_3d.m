%% Calculating LQR feedback parameters for the linearized 3d edge balancing problem
%For simulation, use "run_simulation_2D" 

%clear all; 

%Load the cube parameters 
cubeparameters; 

%Rename for  readability 
mc      = cube.m_tot; 
mw      = wheel.m;
r       = cube.r;
Ic      = cube.Ic; 
Iw0     = wheel.Iw0; 
kt      = motor.kt; %DEBUG: Seems that current become too large

theta0  = atan(sqrt(2)); % Linearization point
g0      = 9.81;          % Gravity

%% Continous system matrices 

%A = [0,0,(-1).*2.^(1/2).*g0.*Ic.^(-1).*(mc+2.*mw).*r.*cot(theta0),0,0, ...
%  0;0,2.^(-1/2).*g0.*Ic.^(-1).*(mc+2.*mw).*r.*(2.^(1/2).*cos(theta0) ...
%  +2.*sin(theta0)),0,0,0,0;0,0,2.^(1/2).*g0.*Ic.^(-1).*(mc+2.*mw).* ...
% r.*csc(theta0),0,0,0];
   A = [0,0,(-1).*2.^(1/2).*g0.*(mc+2.*mw).*r.*(Ic+2.*Iw0+(3.*mc+5.*mw).* ...
  r.^2).^(-1).*cot(theta0),0,0,0;0,2.^(-1/2).*g0.*(mc+2.*mw).*r.*( ...
  Ic+2.*Iw0+(3.*mc+5.*mw).*r.^2).^(-1).*(2.^(1/2).*cos(theta0)+2.* ...
  sin(theta0)),0,0,0,0;0,0,2.^(1/2).*g0.*(mc+2.*mw).*r.*(Ic+2.*Iw0+( ...
  3.*mc+5.*mw).*r.^2).^(-1).*csc(theta0),0,0,0];

%This a only models the highest derivatives, augment to include lower
%derivatives

A = [ zeros(3,3), eye(3)  ; A];


B = [2.^(-1/2).*(Ic+2.*Iw0+(3.*mc+5.*mw).*r.^2).^(-1).*(Ic+2.*(Iw0+ ...
  mw.*r.^2)).^(-1).*(Ic+2.*(Iw0+(mc+2.*mw).*r.^2)).*csc(theta0),2.^( ...
  -1/2).*(Ic+2.*Iw0+(3.*mc+5.*mw).*r.^2).^(-1).*(Ic+2.*(Iw0+mw.* ...
  r.^2)).^(-1).*(Ic+2.*(Iw0+(mc+2.*mw).*r.^2)).*csc(theta0),2.^(1/2) ...
  .*(mc+mw).*r.^2.*(Ic+2.*Iw0+(3.*mc+5.*mw).*r.^2).^(-1).*(Ic+2.*( ...
  Iw0+mw.*r.^2)).^(-1).*csc(theta0);2.^(-1/2).*(Ic+2.*Iw0+(3.*mc+5.* ...
  mw).*r.^2).^(-1),(-1).*2.^(-1/2).*(Ic+2.*Iw0+(3.*mc+5.*mw).*r.^2) ...
  .^(-1),0;(1/2).*(Ic+2.*Iw0+(3.*mc+5.*mw).*r.^2).^(-1).*(Ic+2.*( ...
  Iw0+mw.*r.^2)).^(-1).*(2.*(mc+mw).*r.^2+(-1).*2.^(1/2).*(Ic+2.*( ...
  Iw0+(mc+2.*mw).*r.^2)).*cot(theta0)),(1/2).*(Ic+2.*Iw0+(3.*mc+5.* ...
  mw).*r.^2).^(-1).*(Ic+2.*(Iw0+mw.*r.^2)).^(-1).*(2.*(mc+mw).*r.^2+ ...
  (-1).*2.^(1/2).*(Ic+2.*(Iw0+(mc+2.*mw).*r.^2)).*cot(theta0)),(Ic+ ...
  2.*Iw0+(3.*mc+5.*mw).*r.^2).^(-1).*(Ic+2.*(Iw0+mw.*r.^2)).^(-1).*( ...
  Ic+2.*Iw0+mc.*r.^2+3.*mw.*r.^2+(-1).*2.^(1/2).*(mc+mw).*r.^2.*cot( ...
  theta0))];

B = [zeros(3,3); B];

%We measure all
C = eye(6); 

inputnames ={'T1','T2','T3}'}; 
statenames = {'phi', 'theta', 'psi','phidot' , 'thetadot', 'psidot'};


sys_c = ss(A,B,C,[], 'Inputname',inputnames, 'Statename',statenames);

%% System discetization
Ts = Ts.controller;  %Sampling time of choice 
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
      
Qx = diag([10 10 10 .1 .1 .1]);   %Penalties on states, we care mostly about the angle [60 200 200 1 5 5]
Ru = 1*eye(3);   %Voltage is our only input

[K_lqr_3D,~,~] = lqr(sys_d,Qx,Ru)

%eigenvalues = abs(eig(sys_d.A-sys_d.B*K_lqr))

%% Step response 

%x0 = [0.1,0.1,0.1,0,0,5]

%save('K_lqr_3D','K_lqr_3D')


