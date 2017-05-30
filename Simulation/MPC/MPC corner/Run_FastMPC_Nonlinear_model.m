%% Clear all
clearvars, clc, clf, close all

run cubeparameters.m

[MPC_3d, fMPC_3d, sys_d] = MPC_Parameters_3D(cube, motor ,Ts, wheel);

%% Set up init quaternion orientation

%This is the offset from the perfect balancing point
theta0      =deg2rad(1.5); 
psi0        =deg2rad(-1.5); 
phi0        =deg2rad(1.5); 
    
%These are the values for which the cube stands perfectly on a corner 
phi00    = 0; 
theta00  = atan(sqrt(2)); 
psi00    = pi/4;  

%Add offsets 
phi     = phi00+phi0; 
theta   = theta00+theta0; 
psi     = psi00+psi0; 

%Try to convert to quaternions 



% Q0 = [q1,q2,q3,q4];  
% Q0 = Q0/norm(Q0); 

%% Offset for center of gravity 
%Define an offset vector, how the center of gravity is offset its nominal
%position 

%Set how much off the center of gravity is 
theta_offs = deg2rad(0); 

%The unit vector defining the direction of the offset vector. Should 
%lie in the (Z,X) plane, perpendicular and thus normal to the vector rcb
%going from the corner to the center of gravity 
e_cog_offs = [-cos(pi/4) ; 0 ; sin(pi/4)];

%I guess you have to draw this one to understand it, hopefulle it is in the
%report 
cog_offs = theta_offs*norm(cube.rcb)*e_cog_offs; 

%% Simulate Model

% Start and Stop time for the simulation
StartTime   =    0;
StopTime    =    2;

% Simulate the nonlinear model 
sim('cube_3d_simulation_model_fast_mpc',[StartTime StopTime]);

%% Rename simulation variable for readability

tvec    = deviaiton_angle.Time;

phi     = deviaiton_angle.Data(:,1);
theta   = deviaiton_angle.Data(:,2);
psi     = deviaiton_angle.Data(:,3);

iref1   = iref.Data(:,1);
iref2   = iref.Data(:,2);
iref3   = iref.Data(:,3);

iter_mpc= iter.Data;

%% Plot angles 

clf, close all
set(0,'defaulttextinterpreter','latex')

figure;
plot(tvec,phi,'-',tvec,theta,'--',tvec,psi,'-.'), grid on

xlabel('Time [s]');
ylabel('Degrees [$^{\circ}$]')

l = legend('$\phi$','$\theta$','$\psi$');%'Offset estimate'); 
set(l,'Interpreter','Latex'); 
title('Euler angles when balancing on corner using Fast MPC')

%% Plot insignal

figure;
plot(tvec,iref1,'-',tvec,iref2,'-',tvec,iref3,'-'), grid on, hold on
plot(tvec, ones(size(iref.time))*motor.Imax, 'k--'); 
plot(tvec, -ones(size(iref.time))*motor.Imax, 'k--'); 

ylim([-4.8 4.8])

xlabel('Time [s]');
ylabel('Ampere [A]')

l = legend('Current reference for motor 1','Current reference for motor 2',...
           'Current reference for motor 3', 'Current limit');
set(l,'Interpreter','Latex'); 
title('Euler angles when balancing on corner using Fast MPC')

%% Fast MPC Convergaence properties

cm = 4*nnz(fMPC_3d.LPD)+6*fMPC_3d.nx*fMPC_3d.N;
disp(['The band m = ' num2str(fMPC_3d.M)]);
disp(['C_m = ' num2str(cm)]);
disp(['Iteration Average = ' num2str(mean(iter))]);
disp(['theta_full = '  num2str(cm*mean(iter)/1829333.375)])
