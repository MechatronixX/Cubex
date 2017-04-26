%% 	MPC in 2d
%   TODO: simulation 

clc, clearvars

%Load the cube parameters 
cubeparameters; 

%Initial state
x0 = [deg2rad(2) 0]';


%% 2D system
A = [0                                           1                              
    cube.m_tot*cube.l_corner2cog*9.81/cube.I_2D             0];                              

B = [0 ; motor.kt/cube.I_2D]; 

C = eye(2);

D =[];


%bPack system 
sys_c = ss(A,B,C,D);

% System discetization
sys_d = c2d(sys_c, Ts.controller);

%% Simulate 
opt = mpcqpsolverOptions('single');
xk = x0;
T=50;      % simulation time
 
yvec=[];
uvec=[];
eTime = [];
for k = 1 : T
    beq  = single(MPC.AA)*xk;
    tic
    [z, ~, ~, ~] = mpcqpsolver(single(MPC.Linv), single(MPC.f'), single(MPC.Ain), single(MPC.bin),...
                             single(MPC.Aeq), single(beq), MPC.iA0, opt);        % Solve MPC 
    eTime = [eTime toc];
    uk=double(z(1));                %Extract the first insignal  
    xk=sys_d.A*xk+sys_d.B*uk;       %Update time 
    yvec=[yvec  sys_d.C*xk];        %Save outsignal 
    uvec=[uvec; uk];                %Save insignal 
end
    
%% Plots 
close all; 
set(0,'defaulttextinterpreter','latex')

tvec=Ts.controller*(1:1:T);

%-------------------------Quadprog MPC
plot(tvec,yvec,'-',tvec,uvec','--.'); grid
ylim([-4 2])
title('Linear model')
xlabel('Time [s]'); 
l = legend('Angle $\theta$ [rad]','Angular rate $\omega$ [rad/s]', 'Input $i$ [A]'); 
set(l,'Interpreter','latex')
