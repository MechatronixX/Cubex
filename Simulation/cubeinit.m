%% Initiation procedure that is called when the simulation starts 

%addpath('../lib')

%% Load constant parameters 
%cubeparameters 

%% Load bus types 
% A bus is similar to a struct in C 
load('cube_2d_bus.mat')
load('BUS_motorStates.mat'); 
%load('states.mat')
load('BUS_2Dstatevector'); 
load('BUS_euler.mat'); 
load('BUS_orientation.mat'); 