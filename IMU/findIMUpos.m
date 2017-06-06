close all;
clear all; 
clc; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Used to find a rotation matrix that makes an IMU appear 
%%% aligned perpendicular in relation to the frame where its mounted.
%%%
%%% The procedure is explained in detail in the report. 
%%%
%%% It uses data where the mounting frame is laying perfectly still and
%%% perfectly flat, so that that the gravity vector in that frame would bee
%%% [0 0 g]. Then the frame is rotated around it's desired x-axis. 
%%% 
%%% If succesful, the rotated gravity vector should be as close as possible
%%% to [0 0 g] and the gyroscope should rotate mainly around the x-axis. 
%%%
%%% Still some todos, among other things that there are some false
%%% soluttions that should be found and discarded automatically. 
%%%
%%%-Conventions 
%%% %Note that data is transformed from the body frame (the cube in this case)
%%% %to the sensor frame, e.g. the IMU frame as 
%%%
%%% Body -> Body' -> Body'' -> Sensor 
%%%
%%% or short form 
%%% 
%%% B -> B' -> B'' -> S
%%%
%%% Some of the results are plotted in intermediate euler frames, indicated
%%% by a prefix superscript, like B^a for some value {a} plotted in the
%%% frame {B}
%%%
%%% By Love Palm 2017
%%%

%% Load and analyze data 
close all; 

addpath('../Datasets')
load('IMU3_data'); 

plot(acc.Data); 
title('Accelerometer raw data ');
figure; 
plot(gyro.Data); 
title('Gyroscope raw data ');


%% From the plot, determine a range for when to analyze accelerometer 
acc_range =4500:7500; 
%gyro_range = 11850:12350; 
gyro_range = 2400:3600; 

acc_still   =  acc.Data(acc_range,:); 
gyro_rot    =  gyro.Data(gyro_range,:); 

acc_raw  =  acc.Data; 
gyro_raw =  gyro.Data; 

%Get gravity vector expressed in the IMU sensor frame denoted s 
a_s = mean(acc_still)';

%Should roughly be 9.82 if SI units are used. 
g_norm = norm(a_s);

%The g vector should look like this when expressed in the sensor frame (the
%accelerometer rather measures normal forces) 
g = [0 ; 0; g_norm] 

%% Elementary rotation matrices
%There are matlab packages for this but they require downloading etc so we
%put theme here instead

rot_X= @(a)[ 1      0            0; 
            0      cos(a)      -sin(a) ; 
            0      sin(a)      cos(a) ];
        
rot_Y =@(a) [ cos(a)     0       sin(a); 
             0           1       0 ; 
             -sin(a)      0       cos(a) ];       
         
         
  rot_Z =@(a) [cos(a)   -sin(a)     0;
               sin(a)   cos(a)      0; 
               0        0           1]; 

%% Debug 
T = [1;2;3]
theta_T = atan2(T(2), T(3));
rad2deg(theta_T)

T = rot_X(theta_T)*T

%% Get first rotation from IMU frame (Xs, Xs, Xs) -> (XB'',YB'',ZB'') frame 
phi_x =  atan2(a_s(2), a_s(3));

disp(['Phi_x: ', num2str(rad2deg(phi_x))])
     
a_bprimprim = rot_X(phi_x)*a_s

%% Get rotation around Y axis from (Xb'',Yb'', Zb'') -> (Xb', Yb', Zb') frame 
theta_y = atan2(a_bprimprim(1), -a_bprimprim(3));

a_bprim = rot_Y(theta_y)*a_bprimprim

%TODO: Does the method of inverting both arguments in atan2() always work to get other solution?? 
if( a_bprim(3) <0 )
    disp('Wrong sign during Y rotation');
    theta_y = atan2(-a_bprimprim(1), a_bprimprim(3));
    a_bprim = rot_Y(theta_y)*a_bprimprim
%     %theta_y = theta_y+pi; 
%     Cy = cos(theta_y);
%     Sy = sin(theta_y); 
end


%disp(['True Y rot: ', num2str( rad2deg(IMUrot_true(2))),'   Estimated: ', num2str(rad2deg(theta_y)) ])

rotY = rot_Y(theta_y); 
 
 %% Gyroscope - for correction in the x-y plane 
 
 euler_YX       =  rot_Y(theta_y)*rot_X(phi_x);
 gyro_trans     = (euler_YX*gyro_rot')'; 
 
 gyro_mean = mean(gyro_trans)
 
%  %theta_z = atan2(-gyro_mean(2), -gyro_mean(1))
 %theta_z = pi/4; 

 %Express gyro in an intermediate frame. 
 gyro2 = (euler_YX*gyro.Data')'; 
 
%  %Old approach, uses a gradient decent approach
%  theta_z = findPSI(gyro2(:, 1:2), pi); 

%Analytical approach
 Sx     = sum(gyro_rot(:,1).^2); 
 Sy     = sum(gyro_rot(:,2).^2); 
 Sxy    = sum( gyro_rot(:,1).*gyro_rot(:,2)); 


%theta_z
theta_z2 = 0.5*atan2(-2*Sxy, Sx-Sy)

 
theta_z = theta_z2+pi; 
 
  rotZ = rot_Z(theta_z);
     
%% Complete rotation matrix that makes all IMU measurement appearing in the right frame  
 euler_ZYX = rotZ*euler_YX; 
 
 %% Plot transformed results 

 
 close all; 
 set(0,'defaulttextinterpreter','latex')
 
% acc_transformed = (euler_YX*acc_still')';
% figure; 
% suptitle('Accelerometer');

%subplot(1,2,1);

%------------------------------------------
% Plot accelerometer  
figure; 

subplot(1,2,1); 
plot(acc_still)
l = legend('$^Sa_x$','$^Sa_y$','$^Sa_z$'); 
title('Before transformation'); 
set(l,'Interpreter','Latex','FontSize',12);
%set(l,)

subplot(1,2,2); 
%figure; 
plot((euler_YX*acc_still')'); %The accelerometer data expressed in an intermediate frame 
l = legend('$^{B\prime} a_x$','$^{B\prime} a_y$','$^{B\prime} a_z$'); 
set(l,'Interpreter','Latex','FontSize',12);
title('After transformation')

suptitle('Accelerometer');

%-------------------------------------
%Gyroscope 

figure; 
subplot(1,2,1); 
plot(gyro.Data); 
l = legend('$^{B\prime} \omega_x$','$^{B\prime} \omega_y$','$^{B\prime} \omega_z$'); 
set(l,'Interpreter','Latex','FontSize',12);
title('Before transformation')


%figure; 
subplot(1,2,2); 
plot((euler_ZYX*gyro.Data')'); 
l = legend('$^{B} \omega_x$','$^{B} \omega_y$','$^{B} \omega_z$'); 
set(l,'Interpreter','Latex','FontSize',12);
title('After transformation')
suptitle('Gyroscope');

%% Save values to use for simulation of magdwick etc 
gyro_T = (euler_ZYX*gyro_raw')'; 
acc_T  = (euler_ZYX*acc_raw')';




 