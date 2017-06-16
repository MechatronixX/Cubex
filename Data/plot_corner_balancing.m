% Plot script for balancing on corner
% Works for both LQR 

% Load data
clearvars
load Fast_MPC_Balancing_edge.mat

% Rewrite the parameters
tvec = cube_angle.Time;
angle = cube_angle.Data;
i = iref.Data;
omega = wx.Data;
COG = cog.Data;

% Set the start time and end time for the plots
start_Time = 22;

tvec = tvec - start_Time*ones(length(cube_angle.Time),1);

end_Time = 10;
%% Plot states
close all; 
set(0,'defaulttextinterpreter','latex')

figure;
plot(tvec,angle,'-',tvec,omega,'-'); grid
xlim([0 end_Time])
ylim([-4 1])
title(['States of the system when balancing on edge using ',  controller])
xlabel('Time [s]'); 
l = legend('Angle $\theta$ [$^{\circ}$]','Angular rate $\omega$ [rad/s]'); 
set(l,'Interpreter','latex')

%% Plot insignal [iref]

figure;
plot(tvec,i,'r'); grid
xlim([0 end_Time])
title(['Input signal when balancing on edge using ',  controller])
xlabel('Time [s]'); 
l = legend('Input $i$ [A]'); 
set(l,'Interpreter','latex')


%% Plot cog47.65

figure;
plot(tvec,COG,'-'); grid
xlim([0 end_Time])
title(['COG finder when balancing on edge using ',  controller])
xlabel('Time [s]'); 
l = legend('COG finder [$^{\circ}$]'); 
set(l,'Interpreter','latex')