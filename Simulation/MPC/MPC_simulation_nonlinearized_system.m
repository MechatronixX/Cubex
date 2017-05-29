%MPC Simulation on nonlinear system

clearvars, clc

%Load the cube parameters 
cubeparameters; 

%Initial state
x0 = [deg2rad(2) 0]';

StartTime   =    0;
StopTime    =    5;

% Simulate the nonlinear model 
sim('cube_2d_simulation_model_fast_mpc',[StartTime StopTime]);

%% Plots 

close all; 
set(0,'defaulttextinterpreter','latex')

%-----------------------------------Angle and angular velocity
t = simTime.data(:); 
plot(t, rad2deg(cube_states.cube_angle.data(:) ),'k'); 
hold on
ylabel('Degrees [$^{\circ}$]')
grid on; 
plot(cube_states.cube_angular_velocity.Time, rad2deg(cube_states.cube_angular_velocity.data(:) ), 'r--'); 

l = legend('Angle','Angular rate');%'Offset estimate'); 
set(l,'Interpreter','Latex'); 
xlabel('Time [s]');
title('States of the system using Fast MPC')
%--------------------Current 
figure; 

current = cube_states.motorStates.current.data(:); 
t       = cube_states.motorStates.current.Time(:);

plot(t, current, 'b'); 
hold on; 
grid on; 
plot(iref.time, iref.data(:),'--b'); 
plot(iref.time, ones(size(iref.time))*motor.Imax, 'k--'); 
plot(iref.time, -ones(size(iref.time))*motor.Imax, 'k--'); 
ylabel('Ampere [A]')
l = legend('Current', 'Current reference', 'Current limit'); 
set(l,'Interpreter','Latex'); 
xlabel('Time [s]');
ylim([-4.8 4.8])
title('Input signal using Fast MPC')

%%
cm = 4*nnz(fMPC.LPD)+6*fMPC.nx*fMPC.N;
disp(['The band m = ' num2str(fMPC.M)]);
disp(['C_m = ' num2str(cm)]);
disp(['Iteration Average = ' num2str(mean(iter))]);
disp(['theta_full = '  num2str(cm*mean(iter)/57943.9492)])