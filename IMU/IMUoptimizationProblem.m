close all;
clear all; 

%Use data from IMU that has its horizontal plane correct. That means there
%shouldnt be much rotation around the Z axis 
load('gyro_transformed.mat'); 


plot(gyro2)

%Choose relant data. 
gyro = gyro2(10000:18000,:)


%Number of variables 
N = size(gyro,1); 

%% Cost function 

%Cost function. All elements are measurements except for last elements
%
% x(end-1) = cos(theta_z)
% x(end)   = sin(theta_z) 

fun = @(x) sum(     (x(1:N).*x(end)...    
                    + x(N+1:2*N).*x(end-1) ).^2       );  
                
%The cost function can be tried by manually entering differnt values for theta_z  
theta_z = pi/5.66

x = [gyro(:,1); gyro(:,2); cos(theta_z) ; sin(theta_z)];
tralal = fun( x  )

%% Constraints 
%Linear equality constraints 
% Aeq = [eye(2*N)         zeros(2*N,2) ; 
%        zeros(2,2*N)     zeros(2,2)  ];

%Is our measurements. 
beq = [gyro(1:N,1);gyro(1:N,2); 0 ;0]

%Linear inequality constraints 
A =[]; 
b=[]; 

%Bounds 
lb = []; 
ub = []; 

x0 = beq; 
x0(end-1) = cos(pi/2); 
x0(end)   = sin(pi/2); 

%DOESN'T WORK, WHY? 
%Non linear constraint 
% nonlcon = @nonlinconst;
% 
% x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon); 

%% Try succesive approx in a loop 

theta = 0; 
 x = [gyro(:,1); gyro(:,2); cos(theta) ; sin(theta)];
 
 step = 0.1; 
 dir = 1; 

lastC = inf; 

iter = 1000; 
% Cvec       = zeros(iter,1); 
% THETA   = zeros(iter, 1); 
Cvec = []; 
THETA = []; 

tol = 10^-3; 

for i = 1:iter
    theta = theta + dir*step; 
    
    x(end) = sin(theta);
    x(end-1) = cos(theta); 
    C = fun(x); 
    
    %Take a smaller step backwards if it got worse, else go a bigger step
    %forward
    if C> lastC 
        step = step*0.5; 
        dir = dir*-1; 
    else 
        step = step*1.2; 
    end
    
 
    
    %Save some values 
    
%     THETA(i,:) = theta; 
%     Cvec(i,:) =C;  
    THETA = [THETA; theta];
    Cvec = [Cvec; C]; 
    
    if( abs(C-lastC) <tol ) 
        disp(['Converged! Found a rotation around the z axis ', num2str(rad2deg(theta)),' degrees']); 
        break;
    end
    
       lastC = C; 
end 


%% Plots 
close all; 

%Optimization problem 
figure; 
title('Optimization'); 

plot(Cvec);
yyaxis right; 
plot(THETA); 
legend('Cost','Angle \psi'); 
xlabel('Iteration number'); 

%Use the theta_z from optimization loop
theta_z = theta; 


 rotZ = [cos(theta_z)   -sin(theta_z)   0;
         sin(theta_z)   cos(theta_z)    0; 
         0              0               1]; 
     
  gyro_rotated = (rotZ*gyro')'; 
  
  figure; 

suptitle('Gyro');

subplot(1,2,1); 
plot(gyro)
legend('x','y','z'); 
title('Raw'); 


subplot(1,2,2); 
plot(gyro_rotated); 
legend('x','y','z'); 
title('Transformed')




