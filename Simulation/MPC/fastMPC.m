%% Approximate inverses in fast dual proximal gradient method
% Algorithm design by Emil Klintberg and Sebastien Gross
% Written by David Wall and Love Palm

%% Clear and read

clc, clearvars

%Load the cube parameters 
cubeparameters; 

clearvars -except  Ts cube motor  

[MPC, fMPC, sys_d] = MPC_Parameters(cube, motor, Ts);
%Initial state
x0 = [deg2rad(2) 0]';

% Simulation time
sec = 3;
T = (1/Ts.controller) * sec;

%% Simulation
opt = mpcqpsolverOptions('single');
xk = x0;
lam(:,2) = zeros(length(fMPC.dd),1);
yvec=[];
uvec=[];
eTimeFASTMPC=[];
eTimeMPCSOLVER = [];
for k = 1 : T
    
    beq  = single(MPC.AA)*xk;
    mpcsol=tic;
    [z, ~, ~, ~] = mpcqpsolver(single(MPC.Linv), single(MPC.f'), single(MPC.Ain), single(MPC.bin),...
                             single(MPC.Aeq), single(beq), MPC.iA0, opt);        % Solve MPC 
    eTimeMPCSOLVER = [eTimeMPCSOLVER toc(mpcsol)];
    
    
    d = fMPC.dd * xk;
    lam(:,2) = lam(:,end);
    i = 2;
    fmpc=tic;
    while (i < 100) 
        beta =  (i-3)/i;
        mu = lam(:,i) + beta*(lam(:,i)-lam(:,i-1));
        KK = [fMPC.inCo fMPC.miHDtPt*mu];
        w = median(KK,2);
        lam(:,i+1) = mu + (fMPC.LPD * w) - (fMPC.LP * d);
        i = i + 1;
        if norm((fMPC.D*w)-d,inf) <= 0.001
            break;
        end
    end
    eTimeFASTMPC=[eTimeFASTMPC toc(fmpc)] ;
    
    uk = w(1);
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
%ylim([-4 1])
title('Fast MPC')
xlabel('Time [s]'); 
l = legend('Angle $\theta$ [rad]','Angular rate $\omega$ [rad/s]', 'Input $i$ [A]'); 
set(l,'Interpreter','latex')

%%
figure;
set(0,'defaulttextinterpreter','latex')
plot(eTimeFASTMPC)
grid on, hold on
plot(eTimeMPCSOLVER)

title('Computational time per iteration')

l = legend('Fast-MPC','MPC QP Solver'); 
set(l,'Interpreter','latex')

%% Heat map over P matrix
figure;
set(0,'defaulttextinterpreter','latex')
imagesc(abs(fMPC.P))
colormap(flipud(colormap('gray')))
title('Heat map over P matrix')


xlab = xlabel('Column'); 
ylab = ylabel('Row'); 
set(xlab,'Interpreter','latex')
set(ylab,'Interpreter','latex')
