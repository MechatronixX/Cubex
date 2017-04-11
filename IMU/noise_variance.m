clear all;
close all; 

%We connected two IMU's instead of just one 2017-04-11 with the purpose of
%decreasing noise variance. This holds in theory, see if it's correct in
%practice too. 
addpath('../Datasets')
load('2d_balancing_data') 

%% Check variances 
range = 1:20000; 

var1= var(wx_imu1.Data(range))
ex1 = mean(wx_imu1.Data(range))
var3 = var(wx_imu3.Data(range))
ex3 = mean(wx_imu3.Data(range))
ex_comb = mean((wx_imu1.Data(range)+wx_imu3.Data(range))/2)
var_comb = var((wx_imu1.Data(range)+wx_imu3.Data(range))/2)

%% Plots 
close all; 
set(0,'defaulttextinterpreter','latex')

avg_rng = 1:60000; 

plot(wx_imu1.Data(avg_rng),'b')
hold on;
plot(wx_imu3.Data(avg_rng),'r')
plot((wx_imu1.Data(avg_rng)+wx_imu3.Data(avg_rng))*0.5, 'k', 'LineWidth',3);

ax= gca; 

%TODO: Finda simple way to make all axis values integers 
%ax.XTick = 0:1000:60000; 
%ax.XT
l = legend('IMU 1', 'IMU 2','Average');
set(l,'Interpreter','Latex')

ylabel('[Rad/s]');
xlabel('Sample index [k]')

%figure; 

%plot(wx_imu3)





