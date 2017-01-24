%% Variables
maxvinkel = pi/4;

% vikter  (uppskattade CAD)
Mwheel = 0.273; 
Mcub = 2.083-Mwheel;                  %3-Mwheel;  

% tröghetsmoment (uppskattade CAD)
I_total = 0.035;          %0.052;
I_wheel = 8*10^-4;
I_o = I_total-I_wheel; 

% friktioner (uppskattade tidigare arbeten)
cub_friction =  0.0001; 
wheel_friction = 0.05*10^-3;
my_wheelpad = 2*0.3;

%accelerations
g = 9.81;
Brake_dacc = (4800*0.12*pi)/(60*0.0196);

% storlek
mass_centrum_oneside =  0.085;                %Calculated to be able to walk
Lcub =      sqrt((mass_centrum_oneside^2)*2) -0.085;    %Distance between pivot and center of mass of cube  
Lwheel =    sqrt((0.1^2)*2) -0.1;          %Distance between wheel centra and cube turning corner (pivot)
radius_w = 0.06;

%Motor variables: 
L = 0.463*10^-3; 
R = 0.608;
Kt = 38*10^-3;
Kv = 36.9*10^-3;
I_motor = 1.81*10^-5;
Load = 6*10^-4; 

% Linearization around (0,0,0) %

Km = 36.9*10^-3;
Lt = Lcub;
mt = Mcub+0.275;
g = 9.81;
Fcube = 0.0001;
Fwheel = 0.05*10^-3;
Itot = 0.052;
Iw = 8*10^-4;
Iframe = Itot-Iw; 

% states: angle cube, angle velocity cube, angle velocity wheel

% linearized A matrix
A = [0 1 0; Lt*mt*g/(Itot-Iw) -Fcube/(Itot-Iw) Fwheel/(Itot-Iw); -Lt*mt*g*Iw/((Itot-Iw)*Iw) -Fcube*Iw/((Itot-Iw)*Iw) -Fwheel*Itot/((Itot-Iw)*Iw) ];
B = [0;-Km/Iframe;Km*Itot/(Iframe*Iw)];
C = [1 0 0; 0 1 0; 0 0 1];
D = 0;

% test = ss(A,B,C,D);
% G = tf(test)

%% LQR
Ts = 0.002; 
% Integtral states: Adds integral states at angle cube and angle velocity
% wheel. 
C1 = [1 0 0]; 
C2 = [0 0 1]; 

A_I = [A zeros(3,2); C1 0 0; C2 0 0]; 
B_I = [B; zeros(2,1)];
C_I = [C [0 0; 0 0; 0 0]]; 
D_I = [0 0 0]'; 

% sysd = c2d(ss(A_I,B_I,C_I,D_I),Ts); 

%%
Qx = diag([1 0.8 0.4 0.1 0.1]);        %Qx = diag([10 2 0.1 0.1 0.01]);                                           %diag([6e4 1 1 4e2 1]); 
Qu = 1;

% [K,S,e] = dlqr(sysd.A,sysd.B,Qx,Qu,0);

% kp = K(1:3)
% ki = K(4:5) 
 
%% Kalman filter variables %%
F = [0,0;0,0;0,1];
D =[0,0,0;0,0,0;0,0,0];

R1= eye(3)*1;                          %Covariance for measurment noise pxp
Q=  eye(3)*1;                        %Covariance matrix for process noise nxn
% sys = ss(A,[B,F],C,D); 
Ts = 0.001;

% [kest,LL,P,M,Z] = kalmd(sys,Q,R1,Ts);  

% ST: Har ej Control System Toolbox. Kommenterat ut de funktioner som
% tillhöra denna.



