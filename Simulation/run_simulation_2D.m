%% Calculating LQR feedback parameters for the linearized 2d edge balancing problem

clc; 
clear all; 
close all; 

addpath('../Libraries')
addpath('../Libraries/Magdwick')
addpath('../Libraries/Magdwick/quaternion_library')



%Load the cube parameters 
cubeparameters; 

%LQR feedback matrix 
load('K_lqr.mat')


%% Simulation
%Simulate the discretized closed loop system

%The center of gravity offset in degrees 
cog_offs = 0.2; 

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
plot(t, rad2deg(cube_states.cube_angle.data(:) ), 'k'); 
hold on; 
plot(cog_offs_est.Time, rad2deg(cog_offs_est.Data), 'k--'); 
ylabel('[Degrees]')
grid on; 
yyaxis right 
plot(cube_states.cube_angular_velocity.Time, rad2deg(cube_states.cube_angular_velocity.data(:) ), 'r'); 
ylabel('[rad/s]')
l = legend('Angle measurement','Offset estimate','Angular velocity (rad/s)'); 
set(l,'Interpreter','Latex'); 
xlabel('Time[s]');

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

l = legend('Current', 'Current reference', 'Allowable current'); 
set(l,'Interpreter','Latex'); 
xlabel('Time[s]');

title('Response to initial conditions using LQR control')

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

plot(cog_offs_est.Time, rad2deg(cog_offs_est.Data), 'r'); 
hold on; 
plot(cog_offs_est.Time, ones(size(cog_offs_est.Data))*cog_offs, 'k--');

l = legend('Estimated', 'Real');
title('Offset to center of gravity')
set(l,'Interpreter','Latex'); 











