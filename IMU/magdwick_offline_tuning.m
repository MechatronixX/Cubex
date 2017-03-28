
clear all; 
close all;
clc; 

%Load data for the 2D case where all rotation is happening around the
%x-axis(plot data and verify that is true!) 
load('IMU_transformed_data.mat')


%Steady state range 
r_s = 1:8000; 

%Magdwick 
addpath('../Libraries/Magdwick')
addpath('../Libraries/Magdwick/quaternion_library')

Ts      = 1/500; 
beta    = 0.09; %Larger betas makes it listen more to the accelerometer?


AHRS = MadgwickAHRS('SamplePeriod', Ts, 'Beta', beta);

 bias = struct ('wx',0,...
                'wy',0,...
                'wz',0); 
            
bias.wx = mean(gyro_T(r_s, 1));

%Concluded 2017-03-28 that the measurements of the angular velocity and
%angle are too noisy to be suitable for the control system design. Decided
%to implement some sort of filter to redeem this 

A = [1       0 ;
     0       Ts];   

Nx = 2; 
Ny = 1; 

H = [1 0]; 

xhat    = zeros(Nx,1); 
y       = 1; 
P       = zeros(Nx, Nx); 
Q0      = eye(Nx); 
R0      = 10; 

omega_x = gyro_T(1,1); 

N = length(acc_T); 

quaternion = zeros(N, 4);

for k =2:N
    
    %Its a 2d rotation around the X axis. 
    AHRS.UpdateIMU([gyro_T(k,1)-bias.wx, 0, 0],...
                   [0, acc_T(k,2:3)] );	
% 
%     AHRS.UpdateIMU([gyro_T(k,1)-bias.wx, 0, 0],...
%                    [0.001, 0.001, 10] );	
               
%    AHRS.UpdateIMU([0, 0, 0],...
%                   [0, acc_T(k,2:3)] );	
            
    quaternion(k, :) = AHRS.Quaternion;
        
  %% Generate the measurements and states
  y =  gyro_T(k,1); 
  
  %% Prediction
  xhat  = A*xhat; 
  P     = A*P*A'+Q0;  
  
  %% Update
  S = H*P*H'+R0;
  V = y-H*xhat;
  K = P*H'/S;
  
  P = P-K*S*K';
  
  xhat= xhat+K*V;
  
  %Save angular velocity estimate
  omega_x = [omega_x ; xhat(1)]; 
end 


euler = quatern2euler(quaternion); 

disp(['1000*Variance of euler angle phi in steady state range: ', num2str( 1000*var(euler(r_s,1)) ) ])

 %% Plot transformed results 
 close all; 
 set(0,'defaulttextinterpreter','latex')
 t = (1:N)*Ts;

%subplot(1,2,2); 
figure; 
plot(t,acc_T); 
l = legend('$^{B} a_x$','$^{B} a_y$','$^{B} a_z$'); 
set(l,'Interpreter','Latex','FontSize',12);
%title('Transformed')

figure; 
plot(t,gyro_T); 
hold on; 
plot(t, omega_x,'k'); 

l = legend('$^{B} \omega_x$','$^{B} \omega_y$','$^{B} \omega_z$'); 
set(l,'Interpreter','Latex','FontSize',12);
title('Gyroscope transformed')

figure;

plot(t, rad2deg(euler))
l=legend( 'X rotation $\phi$','Y rotation $\theta$','Z rotation $\psi$'  ); 
set(l,'Interpreter','Latex');
title('Euler angles (ZYX sequence)')
ylabel('Degrees'); 
xlabel('Time[s]');