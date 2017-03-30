close all;
clear all; 
clc; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Used to find a rotation matrix that makes an IMU appear 
%%% aligned perpendicular in relation to the frame where its mounted. 
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
load('IMU_data_around_x_2.mat'); 

plot(acc.Data); 
title('Accelerometer raw data ');
figure; 
plot(gyro.Data); 
title('Gyroscope raw data ');


%% From the plot, determine a range for when to analyze accelerometer 
acc_range =1:2000; 
%gyro_range = 11850:12350; 
gyro_range = 2500:12000; 

acc_still   =  acc.Data(acc_range,:); 
gyro_rot    =  gyro.Data(gyro_range,:); 

acc_raw  =  acc.Data; 
gyro_raw =  gyro.Data; 

%Get gravity vector expressed in the imu gram 
g_IMU = mean(acc_still)';

%Should roughly be 9.82 if SI units are used. 
g_norm = norm(g_IMU);

%The g vector should look like this when expressed in the sensor frame (the
%accelerometer rather measures normal forces) 
g = [0 ; 0; g_norm] 


%% Get rotation from IMU frame (x'', y'', z'') -> (x',y',z) frame 

%TODO: Changing around the signs on the atan sometimes gives correct
%solutions, sometimes not. Investigate this 
theta_x =  atan2(-g_IMU(2), -g_IMU(3));

%DEBUG: 
%theta_x = IMUrot_true(3); 

Cx = cos(theta_x);
Sx = sin(theta_x); 

%Express vector in intermediate frame 
az_prim = Sx*g_IMU(2)+Cx*g_IMU(3); 

%There are always 2 solutions for the rotation that are 180 degrees aoart. The rotation may not change
%sign on the z component, and if it did try the other solution. 
%
%TODO: 
%There is for sure a more economical solution to this poblem 
if( sign(az_prim) ~= sign( g_IMU(3)) )
    
    disp('X rotation changed sign'); 
   
    %Take theta for opposite quadrant solution 
    theta_x =  atan2(g_IMU(2), g_IMU(3));
    Cx = cos(theta_x);
    Sx = sin(theta_x); 
end

disp(['Theta_x: ', num2str(rad2deg(theta_x))])
%disp(['True X rot: ', num2str( rad2deg(IMUrot_true(3))),'   Estimated: ', (num2str( rad2deg(theta_x))) ])

%disp([' ', num2str( rad2deg(IMUrot_true(3))),'   Estimated: ', (num2str( rad2deg(theta_x))) ])

rotX = [ 1      0       0; 
         0      Cx      -Sx ; 
         0      Sx      Cx ];

%% Get rotation around Y axis to (x, y, z) frame 
theta_y = atan2(g_IMU(1), az_prim);

Cy = cos(theta_y);
Sy = sin(theta_y); 

az = Sy*g_IMU(1)+Cy*az_prim; 

%TODO: Figure out a way to discard false solutions  
if( sign(az) ~= sign(az_prim))
    disp('Y rotation changed sign'); 
%     %theta_y = theta_y+pi; 
%     Cy = cos(theta_y);
%     Sy = sin(theta_y); 
end


%disp(['True Y rot: ', num2str( rad2deg(IMUrot_true(2))),'   Estimated: ', num2str(rad2deg(theta_y)) ])

rotY = [ Cy     0       -Sy; 
         0      1       0 ; 
         Sy     0       Cy ];

%Was it right? 
g_analytical = rotY*rotX*g_IMU

rotAnalytical = rotY*rotX;

%% Compare with rotation vector.
%The thing is that the rotvec function always yields a correct solution for two axises, but also
%rotates around the z axis which is not desired here. 
rotvec = vrrotvec(g_IMU,g); 
rotvecMat =  vrrotvec2mat(rotvec);

g_FromRotVec = rotvecMat*g_IMU


%Se what euler angles this would give 

%trueRot = rad2deg(IMUrot_true )
 eulerFromRotVec        = rad2deg(rotm2eul( rotvecMat ))
 eulerFromRotAnalytical = rad2deg(rotm2eul(rotAnalytical ))
 
 %% Gyroscope - for correction in the x-y plane 
 
 euler_YX       =  rotAnalytical; 
 gyro_trans     = (euler_YX*gyro_rot')'; 
 
 gyro_mean = mean(gyro_trans)
 
%  %theta_z = atan2(-gyro_mean(2), -gyro_mean(1))
 %theta_z = pi/4; 

 %Express gyro in an intermediate frame. 
 gyro2 = (euler_YX*gyro.Data')'; 
 
 theta_z = findPSI(gyro2(:, 1:2)); 

 %TODO: Its possible to find the gyro rotation analytically, finnish this!  
 Sx     = sum(gyro_rot(:,1).^2); 
 Sxy    = sum( gyro_rot(:,1).*gyro_rot(:,2)); 
 Sy     = sum(gyro_rot(:,2).^2); 
 
 %theta_z2 = 0.5*atan2(Sxy, -(Sx-Sy))
 %theta_z = theta_z2; 

  rotZ = [cos(theta_z)   -sin(theta_z)   0;
         sin(theta_z)   cos(theta_z)    0; 
         0              0               1]; 
     
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




 