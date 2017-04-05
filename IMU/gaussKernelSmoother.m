function [ y ] = gaussKernelSmoother(t, x, sigma )
%Performs gaussian kernel smoothing on the vectors of 
%x - vector matrix with observation
%Sigma  -is the gauss window parameter in seconds, used for tuning
%t - time vector 

[m, n] = size(x); 

y = zeros(m,n); 

for i =1:m
   
    a = -( t(i)-t(:) ).^2./(2*sigma^2);
    
    w = exp(a)*ones(1, n); 
    
    
    y(i,:) = sum(w.*x)./sum(w); 
    
end


end

