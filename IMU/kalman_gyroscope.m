close all;
clear all; 

%Investigate random walk 
A = eye(2); 

C = [1 1]; 

O = [C C*A]; 

rank(O)


% %-------------------------------------------------------------------
% %Investigate model with external moment, using detection matrix from MPC notes  
% 
% A   = 1; 
% B   = 1;  
% Bd = 0;   
% C  = 1;
% Cd = 1;  
% 
% detectionMatrix = [ (1-A)           -Bd ;
%                      C               Cd ]; 
%                 
%                 
%  rank(detectionMatrix)
% 
% %rank([eye(2)-A -Bd])

%Try



