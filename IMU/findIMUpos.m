close all;
clear all; 
clc; 

%TODO: Find analytical solution to the gyro, and integrate with the rest.  

% %Simulate an IMU orientation 
% IMUrot_true = [pi/4, -pi/3, pi+-pi/8]; 
% 
% %Gravity vector in cube frame 
% g = [0 ; 0 ; 9.82]; 
% 
% 
% rotmZYX = eul2rotm(IMUrot_true);
% 
% %Express the g vector in the sensorframe 
% g_IMU = rotmZYX'*g 

%% Load and analyze data 
close all; 
load('IMU_data_around_x.mat'); 

plot([ax.Data, ay.Data , az.Data] ); 
title('Accelerometer raw data ');
figure; 
plot([wx.Data, wy.Data , wz.Data] ); 
title('Gyroscope raw data ');


%From the plot, determine a range for when to analyze accelerometer 
acc_range =1:8000; 
%gyro_range = 11850:12350; 
gyro_range = 1:14000; 


acc  =  [ax.Data(acc_range), ay.Data(acc_range) , az.Data(acc_range)];
gyro =  [wx.Data(gyro_range), wy.Data(gyro_range) , wz.Data(gyro_range)];

acc_raw  =  [ax.Data, ay.Data , az.Data];
gyro_raw =  [wx.Data(:), wy.Data(:), wz.Data(:)];
%Get gravity vector expressed in the imu gram 

g_IMU = [ mean(ax.Data(acc_range)  ); mean(ay.Data(acc_range)  ); mean(az.Data(acc_range)  )]; 

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

%The rotation may not change signs 
if( sign(az) ~= sign(az_prim))
    disp('Y rotation changed sign'); 
    %theta_y = theta_y+pi; 
    Cy = cos(theta_y);
    Sy = sin(theta_y); 
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
 gyro_trans     = (euler_YX*gyro')'; 
 
 gyro_mean = mean(gyro_trans)
 
%  %theta_z = atan2(-gyro_mean(2), -gyro_mean(1))
 %theta_z = pi/4; 

 %Express gyro in an intermediate frame. 
 gyro2 = (euler_YX*[wx.Data, wy.Data , wz.Data]')'; 
 
 theta_z = findPSI(gyro2(:, 1:2)); 

 %Try analytical 
 
 Sx  = sum(gyro(:,1).^2); 
 Sxy = sum( gyro(:,1).*gyro(:,2)); 
 Sy = sum(gyro(:,2).^2); 
 
 %theta_z2 = 0.5*atan2(Sxy, -(Sx-Sy))
 %theta_z = theta_z2; 

  rotZ = [cos(theta_z)   -sin(theta_z)   0;
         sin(theta_z)   cos(theta_z)    0; 
         0              0               1]; 
     
%rotZ = eye(3); 
 euler_ZYX = rotZ*euler_YX; 
 
 %% Plot transformed results 
 close all; 
 set(0,'defaulttextinterpreter','latex')
 
acc_transformed = (euler_YX*acc')';
figure; 
suptitle('Accelerometer');

%subplot(1,2,1);
figure; 
plot(acc)
l = legend('$^Sa_x$','$^Sa_y$','$^Sa_z$'); 
%title('Raw'); 
set(l,'Interpreter','Latex','FontSize',12);
%set(l,)

%subplot(1,2,2); 
figure; 
plot(acc_transformed); 
l = legend('$^{B\prime} a_x$','$^{B\prime} a_y$','$^{B\prime} a_z$'); 
set(l,'Interpreter','Latex','FontSize',12);
%title('Transformed')


figure; 
plot(gyro2); 
l = legend('$^{B\prime} \omega_x$','$^{B\prime} \omega_y$','$^{B\prime} \omega_z$'); 
set(l,'Interpreter','Latex','FontSize',12);
title('Gyroscope')


figure; 
plot((euler_ZYX*gyro')'); 
l = legend('$^{B} \omega_x$','$^{B} \omega_y$','$^{B} \omega_z$'); 
set(l,'Interpreter','Latex','FontSize',12);
title('Gyroscope transformed')





gyro_T = (euler_ZYX*gyro_raw')'; 
acc_T  = (euler_ZYX*acc_raw')';


%Gyroscope 

% figure; 
% 
% suptitle('Gyro');
% 
% subplot(1,2,1); 
% plot(gyro)
% l = legend('x','y','z');
% set(l,'Interpreter','Latex');
% title('Raw'); 
% 
% subplot(1,2,2); 
% plot((euler_ZYX*gyro')'); 
% legend('x','y','z'); 
% title('Transformed')


 