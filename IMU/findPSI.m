function [ psi ] = findPSI( gyro )
%Find the elementary rotation angle psi that minimizez the gyro data around
%the (Z,Y) axises assuming a ZYX euler angles and that the real system only
%was rotated around the X axis. 
%
%TODO: This should be possible to solve for analytically using partial
%derivatives 

%Cost function. Does one elementary rotation on input gyro data
fun = @(gyro, theta) sum(   (gyro(:,1).*sin(theta) + gyro(:,2).*cos(theta) ).^2       );  
                

%Gradient descent optimization 
theta       = 0; %The angle we try in each loop
 
maxIter = 1000;     %Max number we try

Cvec = []; 

tol = 10^-5;   %Cost difference when to stop iterating  
alpha = 10^-4; %Gain factor in the grad descent version. 
grad = 0.1;    %Initial gradient  
lastC = 0;     %Last cost 
                
for i = 1:maxIter
   
    %Try a new value for the angle in the rotation matrix 
    step =  alpha*(-grad);
    theta = theta+ step; 
    
    %Calculate cost 
    C = fun(gyro, theta); 
    
    %Approximate gradient 
    if(i==1)
       %Initial step 
       grad = 0.01*1/alpha;  
    elseif (step ~= 0)
       grad = (C-lastC)/step;
    end
       
    if( abs(C-lastC) <tol ) 
        disp(['Converged! Found a rotation around the z axis ', num2str(rad2deg(theta)),' degrees']); 
        break;
    end
       
    lastC = C; 
end 


psi = theta; 

end

