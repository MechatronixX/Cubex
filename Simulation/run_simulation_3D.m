
close all;
clear all;

cubeparameters; 

addpath('../Libraries/Magdwick/quaternion_library')

%Set up init quaternion orientation


Q0 = [0,0,0,1];  
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
     
 
    
    