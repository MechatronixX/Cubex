%% Calculating LQR feedback parameters for the linearized 2d edge balancing problem

clc; 
clear all; 
close all; 

%Library paths 
addpath('../Libraries')
addpath('../Libraries/Magdwick')
addpath('../Libraries/Magdwick/quaternion_library')

%Load the cube parameters 
cubeparameters; 

%LQR feedback matrix 
%load('K_lqr.mat')
K_lqr = [204.4025 25.7626];


%% Simulation
%Simulate the discretized closed loop system

%The center of gravity offset in degrees 
cog_offs = 0.0; 

%----Initial condition , [angle, angular rate]
x0 = [degtorad(2) ; 0]; 

maxTorque = cube.m_tot*g*cube.l_corner2cog*sin(x0(1));
minCurrent = maxTorque/motor.kt; 

%At the initial condition for the angle, we need this much current
disp(['Min needed current: ', num2str(minCurrent), 'A'])

sim('cube_2d_simulation_model'); 

%% Plots 
%plot(t, x(:,1));

close all; 
set(0,'defaulttextinterpreter','latex')

%-----------------------------------Angle and angular velocity
%plot(t, rad2deg(y(:,1)) );
t = simTime.data(:); 

%yyaxis left
plot(t, rad2deg(cube_states.cube_angle.data(:) ),'k'); 
hold on; 
%plot(cog_offs_est.Time, rad2deg(cog_offs_est.Data), 'k--'); 
ylabel('Degrees [$^{\circ}$]')
grid on; 
plot(cube_states.cube_angular_velocity.Time, rad2deg(cube_states.cube_angular_velocity.data(:) ), 'r--'); 

l = legend('Angle','Angular rate');%'Offset estimate'); 
set(l,'Interpreter','Latex'); 
xlabel('Time [s]');
title('States of the system using LQR control')
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
%plot(iref.time, sat_iref.data(:)); 
ylabel('Ampere [A]')
l = legend('Current', 'Current reference', 'Current limit'); 
set(l,'Interpreter','Latex'); 
xlabel('Time [s]');
ylim([-8 4.8])
title('Input signal using LQR control')

%----------------------Velocity and current
figure; 
w = cube_states.motorStates.rotational_speed.Data; 
t = cube_states.motorStates.rotational_speed.Time;

subplot(2,1,1);
plot(t, w*rpm_,'r'); 
l = legend('Wheel velocity'); 
ylabel('[RPM]');
grid on; 

%yyaxis right;
subplot(2,1,2); 
plot(t, current,'b'); 
ylabel('[A]')
xlabel('Time [s]')
l = legend('Motor current'); 
set(l,'Interpreter','Latex'); 
grid on; 

%----------------Estimation of COG offset
figure; 


plot(cog_offs_est.Time, ones(size(cog_offs_est.Data))*cog_offs, 'k--');
hold on; 
plot(cog_offs_est.Time, rad2deg(cog_offs_est.Data), 'r'); 


l = legend('Actual', 'Batch averaging' );
title('Estimated angular offset to center of gravity')
ylabel('[Degrees]');
set(l,'Interpreter','Latex'); 











