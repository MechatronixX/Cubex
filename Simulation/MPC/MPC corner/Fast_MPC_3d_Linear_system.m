
run cubeparameters.m

clearvars -except cube motor Ts wheel

[MPC_3d, fMPC_3d, sys_d] = MPC_Parameters_3D(cube, motor ,Ts, wheel);

%%

fMPC_3d.P          =   sparse(double(fMPC_3d.P));
fMPC_3d.LPD        =   sparse(double(fMPC_3d.LPD));
fMPC_3d.LP         =   sparse(double(fMPC_3d.LP));
fMPC_3d.D          =   sparse(double(fMPC_3d.D));
fMPC_3d.miHDtPt    =   sparse(double(fMPC_3d.miHDtPt));
fMPC_3d.dd         =   sparse(double(fMPC_3d.dd));
fMPC_3d.s_para     =   double(fMPC_3d.s_para);
fMPC_3d.N          =   double(fMPC_3d.N);
fMPC_3d.nx         =   double(fMPC_3d.nx);
fMPC_3d.mu         =   double(fMPC_3d.mu);

%% Simulation

% Simulation time
sec = 5;
T = (1/Ts.controller) * sec;

%This is the offset from the perfect balancing point
theta0      =deg2rad(2); 
psi0        =deg2rad(2); 
phi0        =deg2rad(2); 

%These are the values for which the cube stands perfectly on a corner 
phi00    = 0; 
theta00  = atan(sqrt(2)); 
psi00    = pi/4;  
phi     = phi00+phi0; 
theta   = theta00+theta0; 
psi     = psi00+psi0; 

%Set how much off the center of gravity is 
%theta_offs = deg2rad(1); 

%The unit vector defining the direction of the offset vector. Should 
%lie in the (Z,X) plane, perpendicular and thus normal to the vector rcb
%going from the corner to the center of gravity 
%e_cog_offs = [-cos(pi/4) ; 0 ; sin(pi/4)];

%I guess you have to draw this one to understand it, hopefulle it is in the
%report 
%cog_offs = theta_offs*norm(cube.rcb)*e_cog_offs; 
cog_offs = zeros(3,1); %debug

%%
%lam = zeros(fMPC_3d.N*fMPC_3d.nx,500);
%x0 = [phi theta psi zeros(1,3)]';
x0 = deg2rad([2,2,2,0,0,0])';

xk = x0;
lam = zeros(fMPC_3d.N*fMPC_3d.nx,2);%fMPC_3d.P * fMPC_3d.dd * xk;
yvec=[];
uvec=[];
eTimeFASTMPC=[];
eTimeMPCSOLVER = [];

sys_d.B = sys_d.B ./ fMPC_3d.s_para;

for k = 1 : T
    
%     beq  = single(MPC_3d.AA)*xk;
%     if fMPC.N < 50
%         mpcsol=tic;
%         %[z, ~, ~, ~] = mpcqpsolver(single(MPC.Linv), single(MPC.f'), single(MPC.Ain), single(MPC.bin),...
%         %                         single(MPC.Aeq), single(beq), MPC.iA0, opt);        % Solve MPC 
%         eTimeMPCSOLVER = [eTimeMPCSOLVER toc(mpcsol)];
%     end
    
    d = fMPC_3d.dd * xk;
    sig = sparse(fMPC_3d.LP * d);
    i = 2;
    fmpc=tic;
    while true 
        beta =  (i-3)/i;
        mu = lam(:,i) + beta*(lam(:,i)-lam(:,i-1));                                                  
        KK = [fMPC_3d.inCo fMPC_3d.miHDtPt*mu];
        w = double(median(KK,2));
        lam(:,i+1) = mu + (fMPC_3d.LPD * w) - sig;
      
        if norm((fMPC_3d.D*w)-d,Inf) <= 1e-5
            iter(k) = i;
            lam(:,1) = lam(:,i);
            lam(:,2) = lam(:,i+1);
            break;
        end
        i = i + 1;
    end
    eTimeFASTMPC=[eTimeFASTMPC toc(fmpc)] ;
    
    uk = [w(1) w(2) w(3)]';
    xk=sys_d.A*xk+sys_d.B*uk;       %Update time 
    yvec=[yvec  sys_d.C*xk];        %Save outsignal 
    uvec=[uvec  uk];                %Save insignal 
end

%% Plots 
close all; 
set(0,'defaulttextinterpreter','latex')

tvec=Ts.controller*(1:1:k);

%-------------------------Quadprog MPC
plot(tvec,yvec(1:3,:),'-',tvec,uvec','--.'); grid
%ylim([-4 1])
title('Fast MPC')
xlabel('Time [s]'); 
l = legend('Angle $\theta$ [rad]','Angular rate $\omega$ [rad/s]', 'Input $i$ [A]'); 
set(l,'Interpreter','latex')

%% Heat map over P matrix
figure;
set(0,'defaulttextinterpreter','latex')
imagesc(abs(fMPC_3d.P))
colormap(flipud(colormap('gray')))
title('Heat map over preconditioner matrix $P$ in corner balancing')

xlab = xlabel('Column'); 
ylab = ylabel('Row'); 
set(xlab,'Interpreter','latex')
set(ylab,'Interpreter','latex')

