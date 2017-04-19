%MPC Simulation on nonlinear system

clearvars, clc

%Initial state
x0 = [deg2rad(2) 0]';

% Simulate the nonlinear model 
sim('cube_2d_simulation_model_mpc',[0 3]);

%% Plot 

close all; 
set(0,'defaulttextinterpreter','latex')

%-------------------------Quadprog MPC
plot(simTime.Data,angle.data,'-'); 
grid on; hold on;
plot(simTime.Data,omega.data,'-');
plot(iref.Time,iref.Data,'--.')
ylim([-4 2])
title('Nonlinear model')
xlabel('Time [s]'); 
l = legend('Angle $\theta$ [rad]','Angular rate $\omega$ [rad/s]', 'Input $i$ [A]'); 
set(l,'Interpreter','latex')
