close all;
clear all; 

%Use data from IMU that has its horizontal plane correct. That means there
%shouldnt be much rotation around the Z axis 
load('gyro_transformed.mat'); 


%plot(gyro2)

%Choose relant data. 
gyro = gyro2(10000:16000,:);


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

%% Try gradient descent in a loop 

theta       = 0; %The angle we try in each loop

%The vector to be passed to the cost function 
 x = [gyro(:,1); gyro(:,2); cos(theta) ; sin(theta)];
 
maxIter = 1000;     %Max number we try

Cvec = []; 
THETA = []; 

tol = 10^-5;   %Cost difference when to stop iterating  
alpha = 10^-4; %Gain factor in the grad descent version. 
grad = 0.1;    %Initial gradient  
lastC = 0;     %Last cost 

findPSI(gyro)

for i = 1:maxIter
   
    %Try a new value for the angle in the rotation matrix 
    step =  alpha*(-grad);
    theta = theta+ step; 
    
    %Calculate cost 
    x(end)      = sin(theta);
    x(end-1)    = cos(theta); 
    C = fun(x); 
    
    %Approximate gradient 
    if(i==1)
       %Initial step 
       grad = 0.01*1/alpha;  
    elseif (step ~= 0)
       grad = (C-lastC)/step;
    end
   
    %Save some values 
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
set(0,'defaulttextinterpreter','latex')

%Optimization problem 
figure; 
title('Optimization'); 

plot(Cvec);
yyaxis right; 
plot(THETA); 
l=legend('Cost','Angle $\psi$'); 
set(l,'Interpreter','Latex');
xlabel('Iteration number'); 

%Use the theta_z from optimization loop
theta_z = theta; 


 rotZ = [cos(theta_z)   -sin(theta_z)   0;
         sin(theta_z)   cos(theta_z)    0; 
         0              0               1]; 
     
  gyro_rotated = (rotZ*gyro')'; 
  
  figure; 

%suptitle('Gyro');

%subplot(1,2,1); 
figure; 
plot(gyro)
l=legend('$^{B \prime}\omega_x$','$^{B \prime}\omega_y$','$^{B \prime}\omega_z$'); 
set(l,'Interpreter','Latex','FontSize',12);
%title('Raw'); 
%xlabel('Sample index'); 

%subplot(1,2,2); 
figure; 
plot(gyro_rotated); 
l = legend('$^B\omega_x$','$^B\omega_y$','$^B\omega_z$'); 
set(l,'Interpreter','Latex','FontSize',12);
%title('Transformed')
%xlabel('Sample index'); 




