% Script for ploting Euler angles and Current referance for LQR corner
% balancing 

clearvars
load Data_corner_Balancing.mat

%% Extract data

% Sample intervall when cube is balacning on corner
StartSample = 6051;
StopSample = 25202;

i = StartSample:StopSample;

% Time vector 
tvec = phi.time(i) - phi.time(StartSample)*ones(length(i),1);

% Euler Angle
phi     = phi.Data(i);
theta   = theta.Data(i);
psi     = psi.Data(i);

% Cog values
COGx    = cogx.Data(i);
COGy    = cogy.Data(i);
COGz    = cogz.Data(i);

% Current referance value
motor1  = iref1.Data(i);
motor2  = iref2.Data(i);
motor3  = iref3.Data(i);

%% Plot angles 

clf, close all
set(0,'defaulttextinterpreter','latex')


figure;
subplot(3,1,1)
plot(tvec,phi,'-'), grid on
ylim([-2 2])

l1 = legend('$\phi$');
title('Estimated Euler angles from balancing on corner using LQR');

subplot(3,1,2)
plot(tvec,theta,'-','Color',[0.8500,0.3250,0.0980]), grid on
ylim([-2 2])

ylabel('Degrees [$^{\circ}$]')
l2 = legend('$\theta$');

subplot(3,1,3)
plot(tvec,psi,'-','Color',[0.9290,0.6940,0.1250]),grid on
ylim([-2 2])

l3 = legend('$\psi$');
xlabel('Time [s]');

set(subplot(3,1,1), 'Position', [0.08, 0.74, 0.9, 0.21])
set(subplot(3,1,2), 'Position', [0.08, 0.43, 0.9, 0.21])
set(subplot(3,1,3), 'Position', [0.08, 0.1,  0.9, 0.21])

set(l1,'Interpreter','Latex'); 
set(l2,'Interpreter','Latex'); 
set(l3,'Interpreter','Latex'); 
 
%% Plot insignal
set(0,'defaultaxesposition','default')

figure;
subplot(3,1,1)
plot(tvec,motor1,'-'), grid on
ylim([-5 5])

l1 = legend('Current reference for motor 1','Location','best');
title('Input signals from balancing on corner using LQR')


subplot(3,1,2)
plot(tvec,motor2,'-','Color',[0.8500,0.3250,0.0980]), grid on
ylim([-5 5])
T =  gca
ylabel('Ampere [A]')
l2 = legend('Current reference for motor 2','Location','best');

subplot(3,1,3)
P3 = plot(tvec,motor3,'-','Color',[0.9290,0.6940,0.1250]), grid on
ylim([-5 5])

l3 = legend('Current reference for motor 3','Location','best');
xlabel('Time [s]');

set(subplot(3,1,1), 'Position', [0.08, 0.74, 0.9, 0.21])
set(subplot(3,1,2), 'Position', [0.08, 0.43, 0.9, 0.21])
set(subplot(3,1,3), 'Position', [0.08, 0.1,  0.9, 0.21])

set(l1,'Interpreter','Latex'); 
set(l2,'Interpreter','Latex'); 
set(l3,'Interpreter','Latex'); 
