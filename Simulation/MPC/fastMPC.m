%% Approximate inverses in fast dual proximal gradient method
% Algorithm design by Emil Klintberg and Sebastien Gross
% Written by David Wall and Love Palm

%% Clear and read

clearvars

%Load the cube parameters 
cubeparameters; 

clearvars -except  Ts cube motor  

[MPC, fMPC, sys_d] = MPC_Parameters(cube, motor, Ts);
%Initial state
x0 = [deg2rad(2) 0]';

% Simulation time
sec = 10;
T = (1/Ts.controller) * sec;



%% Make it to sparse matrices

fMPC.P          =   sparse(double(fMPC.P));
fMPC.LPD        =   sparse(double(fMPC.LPD));
fMPC.LP         =   sparse(double(fMPC.LP));
fMPC.D          =   sparse(double(fMPC.D));
fMPC.miHDtPt    =   sparse(double(fMPC.miHDtPt));
fMPC.dd         =   sparse(double(fMPC.dd));
fMPC.s_para     =   double(fMPC.s_para);

%% Heat map over P matrix
figure;
set(0,'defaulttextinterpreter','latex')
imagesc(abs(fMPC.P))
colormap(flipud(colormap('gray')))
title('Heat map over preconditioner matrix $P$')


xlab = xlabel('Column'); 
ylab = ylabel('Row'); 
set(xlab,'Interpreter','latex')
set(ylab,'Interpreter','latex')
%% Simulation
opt = mpcqpsolverOptions('single');
xk = x0;
lam(:,2) = fMPC.P * fMPC.dd * xk;
yvec=[];
uvec=[];
eTimeFASTMPC=[];
eTimeMPCSOLVER = [];

sys_d.B = sys_d.B ./ fMPC.s_para;

for k = 1 : T
    
%     beq  = single(MPC.AA)*xk;
%     if fMPC.N < 50
%         mpcsol=tic;
%         %[z, ~, ~, ~] = mpcqpsolver(single(MPC.Linv), single(MPC.f'), single(MPC.Ain), single(MPC.bin),...
%         %                         single(MPC.Aeq), single(beq), MPC.iA0, opt);        % Solve MPC 
%         eTimeMPCSOLVER = [eTimeMPCSOLVER toc(mpcsol)];
%     end
    
    d = fMPC.dd * xk;
    sig = sparse(fMPC.LP * d);
    i = 2;
    fmpc=tic;
    while true 
        beta =  (i-3)/i;
        mu = lam(:,i) + beta*(lam(:,i)-lam(:,i-1));
        KK = [fMPC.inCo fMPC.miHDtPt*mu];
        w = double(median(KK,2));
        lam(:,i+1) = mu + (fMPC.LPD * w) - sig;
      
        if norm((fMPC.D*w)-d,Inf) <= 1e-5
            iter(k) = i - 1 ;
            lam(:,2) = lam(:,i+1);
            lam(:,3:end) = [];
            break;
        end
        i = i + 1;
    end
    eTimeFASTMPC=[eTimeFASTMPC toc(fmpc)] ;
    
    uk = fMPC.s_para*w(1);
    xk=sys_d.A*xk+sys_d.B*uk;       %Update time 
    yvec=[yvec  sys_d.C*xk];        %Save outsignal 
    uvec=[uvec; uk];                %Save insignal 
end

%% Plots 
close all; 
set(0,'defaulttextinterpreter','latex')

tvec=Ts.controller*(1:1:T);

%-------------------------Quadprog MPC
plot(tvec,yvec,'-',tvec,uvec./motor.kt','--.'); grid
ylim([-4 1])
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
title('Heat map over preconditioner matrix $P$ in edge balancing')


xlab = xlabel('Column'); 
ylab = ylabel('Row'); 
set(xlab,'Interpreter','latex')
set(ylab,'Interpreter','latex')

% hold on
% for p = 1 : 3
%     x1=0;
%     x2=fMPC.N-(p*20);
%     y1=0;
%     y2=fMPC.N-(p*20);
%     x = [x1, x2, x2, x1, x1];
%     y = [y1, y1, y2, y2, y1];
%     plot(x, y,'LineWidth',2);
% end
% l = legend('$N = 80$','$N = 60$','$N = 40$');
% set(l,'Interpreter','latex')
