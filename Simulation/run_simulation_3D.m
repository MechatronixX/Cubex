
close all;
clear all;

cubeparameters; 

addpath('../Libraries/Magdwick/quaternion_library')

load('K_lqr_3D.mat'); 

%% Set up init quaternion orientation

%This is the offset from the perfect balancing point
theta0       = deg2rad(2); 
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

q1 = cos(psi/2); 

q2 = sin(phi).*sin(theta);
q3 = (-1).*cos(phi).*sin(theta)
q4 = cos(theta);

%This is rquired to get a quaternion 
q2 = q2*sin(psi/2); 
q3 = q3*sin(psi/2); 
q4 = q4*sin(psi/2); 

Q0 = [q1,q2,q3,q4];  
Q0 = Q0/norm(Q0); 

%Run simulation 


 %% Set up visualization
    %Plot every N sample (lower number -> higher frequency)
    %plotEveryMultipleOf = 10;  
  
    visWindow = figure(1);
    visWindow.Units = 'normalized';      %In relation to the screen i believe
    visWindow.Position = [  0 0.1 0.8 0.7];    %[left bottom width height]
   
    subplot(1, 1, 1);
    simAnimation = OrientationView('Simulation', gca);
     title(simAnimation, 'Simulation', 'FontSize', 16);
     
     %Plot every K'th sample for faster runtime 
     K = 10; 
     
     %Set playback speed 
     playbackSpeed = 1; 
      
     for i=1:K:length(orientation.Data)-1
         
            tstart = tic; 
            quaternion = orientation.Data(i,:); 
            setOrientation(simAnimation, quaternion);
                
%             %Textbox 
%             dim = [.2 .5 .3 .3];
%             str = ['T,' num2str(orientation.Time(i))];
%             annotation('textbox',dim,'String',str,'FitBoxToText','on'); 

            tdelay = toc(tstart); 


            %Make sure there is a real time playback
            Tnom = (orientation.Time(i+K)-orientation.Time(i))/playbackSpeed; 

            if(Tnom > tdelay)
                pause(Tnom-tdelay);
            end
          
       
     end
     
 
    
    