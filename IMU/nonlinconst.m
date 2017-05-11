function [c,ceq] = circlecon(x)
%c = (x(1)-1/3)^2 + (x(2)-1/3)^2 - (1/3)^2;

%The constraint is the trigonometric 1
c = x(end)^2 +x(end-1)^2 -1;
ceq = [];