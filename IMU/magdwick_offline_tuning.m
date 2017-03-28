
clear all; 
close all;
clc; 

%Load data for the 2D case where all rotation is happening around the
%x-axis(plot data and verify that is true!) 
load('IMU_transformed_data.mat')


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
