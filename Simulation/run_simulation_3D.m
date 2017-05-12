
close all;
clear all;

%% 


load('K_lqr_3D.mat');  
cubeparameters; 
addpath('../Libraries/Magdwick/quaternion_library')



%% Set up init quaternion orientation

%This is the offset from the perfect balancing point
theta0      =deg2rad(5); 
psi0        =deg2rad(2); 
phi0        =deg2rad(2); 

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
theta_offs = deg2rad(1); 

%The unit vector defining the direction of the offset vector. Should 
%lie in the (Z,X) plane, perpendicular and thus normal to the vector rcb
%going from the corner to the center of gravity 
%e_cog_offs = [-cos(pi/4) ; 0 ; sin(pi/4)]

%I guess you have to draw this one to understand it, hopefulle it is in the
%report 
%cog_offs = theta_offs*norm(cube.rcb)*e_cog_offs; 
cog_offs = zeros(3,1); %debug

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
    
    