
close all;
clear all;
clc;

%% 

load('K_lqr_3D.mat');  
cubeparameters; 
addpath('../Libraries/Magdwick/quaternion_library')

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
%cog_offs = zeros(3,1); %debug

%% Simulate Model

% Start and Stop time for the simulation
StartTime   =    0;
StopTime    =    2;

% Simulate the nonlinear model 
sim('cube_3d_simulation_model',[StartTime StopTime]);

%% Rename simulation variable for readability

tvec    = deviaiton_angle.Time;

phi     = deviaiton_angle.Data(:,1);
theta   = deviaiton_angle.Data(:,2);
psi     = deviaiton_angle.Data(:,3);

iref1   = iref.Data(:,1);
iref2   = iref.Data(:,2);
iref3   = iref.Data(:,3);

iref_sat1   = iref_sat.Data(:,1);
iref_sat2   = iref_sat.Data(:,2);
iref_sat3   = iref_sat.Data(:,3);

%% Plot angles 

clf, close all
set(0,'defaulttextinterpreter','latex')


figure;
plot(tvec,phi,'-',tvec,theta,'--',tvec,psi,'-.'), grid on

xlabel('Time [s]');
ylabel('Degrees [$^{\circ}$]')

l = legend('$\phi$','$\theta$','$\psi$');%'Offset estimate'); 
set(l,'Interpreter','Latex'); 
title('Euler angles of the system using LQR');
 
%% Plot insignal

figure;
plot(tvec,iref_sat1,'-',tvec,iref_sat2,'--',tvec,iref_sat3,'-.'), grid on, hold on
plot(tvec, ones(size(iref.time))*motor.Imax, 'k--'); 
plot(tvec, -ones(size(iref.time))*motor.Imax, 'k--'); 
plot(tvec,iref1,'-','Color',[0,0.4470,0.7410])
plot(tvec,iref2,'--','Color',[0.8500,0.3250,0.0980])
plot(tvec,iref3,'-.','Color',[0.9290,0.6940,0.1250])
ylim([-9 5])

xlabel('Time [s]');
ylabel('Ampere [A]')

l = legend('Current reference for motor 1','Current reference for motor 2',...
           'Current reference for motor 3', 'Current limit');
set(l,'Interpreter','Latex'); 
title('Input signal using LQR control')

%% Covert to quaternions 

% phi = eulerOut.Data(:,1);
% theta = eulerOut.Data(:,2); 
% psi = eulerOut.Data(:,3); 
% 
% q1 = cos(psi./2); 
% q2 = sin(psi./2).*sin(phi).*sin(theta);
% q3 = sin(psi./2).*(-1).*cos(phi).*sin(theta);
% q4 = sin(psi./2).*cos(theta);
% 
% quaternionOrientation = [q1,q2,q3,q4];
% 
% time = eulerOut.Time; 

 %% Set up visualization
%     %Plot every N sample (lower number -> higher frequency)
%     %plotEveryMultipleOf = 10;  
%   
%     visWindow = figure(1);
%     visWindow.Units = 'normalized';      %In relation to the screen i believe
%     visWindow.Position = [  0 0.1 0.8 0.7];    %[left bottom width height]
%    
%     subplot(1, 1, 1);
%     simAnimation = OrientationView('Simulation', gca);
%      title(simAnimation, 'Simulation', 'FontSize', 16);
%      
%      %Plot every K'th sample for faster runtime 
%      K = 10; 
%      
%      %Set playback speed 
%      playbackSpeed = 1; 
%       
%      for i=1:K:length(quaternionOrientation)-1
%          
%             tstart = tic; 
%             %quaternion = orientation.Data(i,:); 
%             
%             quaternion = quaternionOrientation(i,:);
%             setOrientation(simAnimation, quaternion);
%                 
% %             %Textbox 
% %             dim = [.2 .5 .3 .3];
% %             str = ['T,' num2str(orientation.Time(i))];
% %             annotation('textbox',dim,'String',str,'FitBoxToText','on'); 
% 
%             tdelay = toc(tstart); 
% 
% 
%             %Make sure there is a real time playback
%             Tnom = (time(i+K)-time(i))/playbackSpeed; 
% 
%             if(Tnom > tdelay)
%                 pause(Tnom-tdelay);
%             end
%           
%        
%      end
%      
%  
    
    