% Script for ploting Euler angles and Current referance for LQR edge
% balancing 

clearvars -except cog cube_angle iref wx
load data_edge_balancing_fast_mpc.mat

%% Extract data

% Sample intervall when cube is balacning on corner
StartSample = 10821;
StopSample = 16251;

i = StartSample:StopSample;

% Time vector 
tvec = cube_angle.time(i) - cube_angle.time(StartSample)*ones(length(i),1);

% Euler Angle
theta   = cube_angle.Data(i);

% Angular rate
wx      = wx.data(i);

% Cog values
COG     = cog.Data(i);

% Current referance value
iref  = iref.Data(i);

%% Plot angles 

clf, close all
set(0,'defaulttextinterpreter','latex')

figure;
subplot(3,1,1)
plot(tvec,theta,'-'), grid on
ylim([-2 2])

l1 = legend('Angle [$^{\circ}$]');
title('State and input values from the cube when balancing on edge using LQR');

subplot(3,1,2)
plot(tvec,wx.*(180/pi),'-','Color',[0.8500,0.3250,0.0980]), grid on
%ylim([-5 10])


l2 = legend('Angular rate [$^{\circ}$/s]');

subplot(3,1,3)
plot(tvec,iref,'-','Color',[0.9290,0.6940,0.1250]),grid on
ylim([-4 4])

l3 = legend('Current reference [A]');
xlabel('Time [s]');

set(subplot(3,1,1), 'Position', [0.06, 0.74, 0.92, 0.21])
set(subplot(3,1,2), 'Position', [0.06, 0.43, 0.92, 0.21])
set(subplot(3,1,3), 'Position', [0.06, 0.1,  0.92, 0.21])

set(l1,'Interpreter','Latex'); 
set(l2,'Interpreter','Latex'); 
set(l3,'Interpreter','Latex'); 