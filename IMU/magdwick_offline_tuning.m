
clear all; 
close all;
clc; 

%Load data for the 2D case where all rotation is happening around the
%x-axis(plot data and verify that is true!) 
load('IMU_transformed_data.mat')



%Concluded 2017-03-28 that the measurements of the angular velocity and
%angle are too noisy to be suitable for the control system design. Decided
%to implement some sort of filter to redeem this 

A = eye(3); 

Nx = 3; 
Ny = 1; 

H = [0 1 0]; 

xhat    = zeros(Nx,1); 
y       = 1; 
P       = zeros(Nx, Nx); 
Q0      = eye(Nx); 
R0      = eye(Ny); 

N = 1000; 
for k =2:N
    
  %% Generate the measurements and states
  %xhat = A*xhat; 
  
  %% Prediction
  xhat = A*xhat; 
  P = A*P*A'+Q0;  %This is still the last Pkk!  
  
  %% Update
  S = H*P*H'+R0;
  V = y-H*xhat;
  K = P*H'/S;
  
  P = P-K*S*K';
  
  xhat= xhat+K*V;
  
end 





 %% Plot transformed results 
 close all; 
 set(0,'defaulttextinterpreter','latex')
 

%subplot(1,2,2); 
figure; 
plot(acc_T); 
l = legend('$^{B} a_x$','$^{B} a_y$','$^{B} a_z$'); 
set(l,'Interpreter','Latex','FontSize',12);
%title('Transformed')

figure; 
plot(gyro_T); 
l = legend('$^{B} \omega_x$','$^{B} \omega_y$','$^{B} \omega_z$'); 
set(l,'Interpreter','Latex','FontSize',12);
title('Gyroscope transformed')
