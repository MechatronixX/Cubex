
clear all; 
close all;
clc; 

%Load data for the 2D case where all rotation is happening around the
%x-axis(plot data and verify that is true!) 
%load('IMU_transformed_data.mat')

%For this dataset the data has to be rearranged 
load('IMU_data_edgebalancing_2')
gyro_T   = [wx.Data, zeros(size(wx.Data)), zeros(size(wx.Data))]; 
acc_T    = [zeros(size(ay.Data)), ay.Data, az.Data]; 
phi_offs = offset.Data(end);  %In the 2D case an offset is added to the angle 


%% Global parameters 
%Steady state range 
addpath('../Simulation')
cubeparameters; 

r_s = 1:8000; 

fs      = 500; 
Ts      = 1/fs; 

%% Magdwick 
addpath('../Libraries/Magdwick')
addpath('../Libraries/Magdwick/quaternion_library')

beta    = 0.04; %Larger betas makes it listen more to the accelerometer?

AHRS = MadgwickAHRS('SamplePeriod', Ts, 'Beta', beta);

 bias = struct ('wx',0,...
                'wy',0,...
                'wz',0); 
            
%bias.wx = mean(gyro_T(r_s, 1));

%Concluded 2017-03-28 that the measurements of the angular velocity and
%angle are too noisy to be suitable for the control system design. Decided
%to implement some sort of filter to redeem this 

%% Kalman 
A = [1       Ts ;               %State transistion matri (CA)
     0       1];   

Nx = 2; 
Ny = 1; 

H = [1 0];                      %Measurement matrix

xhat        = zeros(Nx,1);      %State estimate
xhat(1,1)   = gyro_T(1,1);      
y           = 0;                %Measurement
P           = zeros(Nx, Nx);    %State covariance
Q0          = eye(Nx);          %State noise
R0          = 3;                %Measurement noise

%omega_x = gyro_T(1,1); 

N = length(acc_T); 

quaternion = zeros(N, 4);

%% Low-pass filters

%Save struct 
omega_x = struct (  'LP_single_pole',   zeros(N,1),...
                    'Gauss',            zeros(N,1),...
                    'LP_double_pole',   zeros(N,1),...
                    'kalman_CA',        zeros(N,1),...
                    'raw',              gyro_T(:,1) ); 


%----First order low pass filter time constant 
Tf      = 0.005;       
alpha   = Ts/(Ts+Tf); 

%----Butterworth
fc                      = 400; 
[b,a]                   = butter(2,fc/fs); 
omega_x.LP_double_pole  = filter(b,a,gyro_T(:,1) )

acc_LP  = filter(b,a, acc_T);  
gyro_LP = filter(b,a,gyro_T);

%% Gauss kernel 
sigma           = 0.01; %Gauss time window, tuning parameter 
omega_x.Gauss   = gaussKernelSmoother((1:N)*Ts, gyro_T(:,1), sigma ); 


%% Kalman filter execution 
omega_x.kalman_CA(1,:) = gyro_T(1,1); 

for k =2:N
    
    %Its a 2d rotation around the X axis. 
    AHRS.UpdateIMU([gyro_T(k,1), 0, 0],...
                   [0, acc_T(k,2:3)] );	
% 
%     AHRS.UpdateIMU([gyro_T(k,1)-bias.wx, 0, 0],...
%                    [0.001, 0.001, 10] );	
               
%    AHRS.UpdateIMU([0, 0, 0],...
%                   [0, acc_T(k,2:3)] );	
            
    quaternion(k, :) = AHRS.Quaternion;
    angles           = quatern2euler(quaternion(k, :)); 
    phi              = angles(1); 
    
        
  %% Generate the measurements and states
  y =  gyro_T(k,1); 
  
  %% Prediction
  
  %For linear kalman 
  xhat  = A*xhat;
 
% %   %For EKF 
%   xhat(2) = sin( phi-deg2rad(phi_offs) )*cube.m_tot*cube.l_corner2cog*9.81/cube.I_2D + motor.kt/cube.I_2D*iref.data(k); 
%   xhat(1) = xhat(2)*Ts +xhat(1); 
  
  P     = A*P*A'+Q0;  
  
  %% Update
  S = H*P*H'+R0;
  V = y-H*xhat;
  K = P*H'/S;
  
  P = P-K*S*K';
  
  xhat= xhat+K*V;
  
  %Save angular velocity estimate
  omega_x.kalman_CA(k) = xhat(1);  
  
  %% First order low pass filter 
  omega_x.LP_single_pole(k) = omega_x.LP_single_pole(k-1)*(1-alpha)+alpha*gyro_T(k,1); 
end 

%The euler angles output are (X, Y ,Z ) order 
euler       = quatern2euler(quaternConj( quaternion)); 
theta_LP    = (filter(b,a, euler(:,1)))'; 

disp(['1000*Variance of euler angle phi in steady state range: ', num2str( 1000*var(euler(r_s,1)) ) ])


%% MSE errors 
%The gaussian smoothed data is seen as ground truth
MSE = struct('SP',0,...
             'DP',0,...
             'Kalman',0,... 
             'Raw',0); 
         
MSE.SP      = sum( (omega_x.Gauss-omega_x.LP_single_pole).^2  );
MSE.DP      = sum( (omega_x.Gauss-omega_x.LP_double_pole).^2  );
MSE.Kalman  = sum( (omega_x.Gauss-omega_x.kalman_CA).^2  );
MSE.Raw     = sum( (omega_x.Gauss-omega_x.raw).^2  )

 %% Plot transformed results 
 close all; 
 set(0,'defaulttextinterpreter','latex')
 t = (1:N)*Ts;

%subplot(1,2,2); 
figure; 
plot(t,acc_T); 
l = legend('$^{B} a_x$','$^{B} a_y$','$^{B} a_z$'); 
set(l,'Interpreter','Latex','FontSize',12);
%title('Transformed')

%-----------------Gyroscope 
figure; 
plot(t,gyro_T(:,1)); 
hold on; 
plot(t, omega_x.kalman_CA,'k'); 
plot(t, omega_x.LP_single_pole); 
plot(t, omega_x.LP_double_pole); 
plot(t, omega_x.Gauss,'k', 'LineWidth',2); 

%l = legend('$^{B} \omega_x$','$^{B} \omega_y$','$^{B} \omega_z$'); 
l = legend('Raw', 'Kalman','1st order lowpass','2nd order lowpass','Gauss smoother'); 
set(l,'Interpreter','Latex','FontSize',12);
title('Gyroscope angular rate $\omega_x$')

%------------Euler angles 
figure;
plot(t, rad2deg(euler(:,1)) - ones(length(euler),1)*phi_offs )
hold on; 
grid on; 

%plot(t, theta_LP); 
l=legend( 'X rotation $\phi - offset $','Y rotation $\theta$','Z rotation $\psi$'  ); 
set(l,'Interpreter','Latex');
title('Euler angles (ZYX sequence)')
ylabel('Degrees'); 
xlabel('Time[s]');

%-------------------Insignal 
figure; 
plot(iref.Data) 
hold on; 
yyaxis right; 
plot(omega_x.raw); 
legend('Current', 'Angular rate'); 
grid; 

figure; 
plot(rad2deg(euler(:,1))- ones(length(euler),1)*phi_offs); 
hold on; 
%yyaxis right; 
plot(omega_x.raw); 
legend('Angle', 'Angular rate'); 
grid; 