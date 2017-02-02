%% Calculating LQR feedback parameters for the linearized 2d edge balancing problem


%Load the cube parameters 
cubeparameters; 

%% Continous system matrices 
A = [0                                          1                           0                        0;
    cube.m_tot*cube.l_corner2cog/cube.I_edge*g  0                           0                        0;
    0                                           0                           0                        motor.kt/wheel.Iy;
    0                                           0                           -motor.kw/motor.L        -motor.R/motor.L];

B = [0 ; 0; 0; -1/motor.L]; 

C  = [1 0 0 0; 
      0 1 0 0;
      0 0 1 0;
      0 0 0 0];
  
D=0; 
 
sys_c = ss(A,B,C,[]); 
sys_c.Inputname ='Vin'; 
sys_c.Statename = {'theta_c'  'omega_c'  'omega_m' 'i' }

%% System discetization
Ts = 0.01;  %Sampling time of choice 
sys_d = c2d(sys_c, Ts)

%% Reachability 
Co = ctrb(sys_d); 
rank(Co)

%% LQR 

Qx = diag([100 1 0.1 0.1]);   %Penalties on states, we care mostly about the angle 
Ru = 1;                       %Voltage is our only input

[K,~,~] = lqr(sys_d,Qx,Ru) 


%% Simulation
%Simulate the closed loop system

close all; 

Acl = [(sys_d.A-sys_d.B*K)];
Bcl = [sys_d.B];
Ccl = [sys_d.C];
Dcl = [];

sys_cl = ss(Acl,Bcl,Ccl,Dcl); 

t = 0:0.01:5;
%r =0.2*ones(size(t));
u=zeros(size(t)); 

[y,t,x]=lsim(sys_cl,u,t, [pi/4 ; 0;0;0] );

%[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
%set(get(AX(1),'Ylabel'),'String','cart position (m)')
%set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')

plot(t, x(:,1)); 

title('Step Response with LQR Control')











