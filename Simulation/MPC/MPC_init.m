%% 	MPC in 2d
%   TODO: simulation 

clc, clearvars

%Load the cube parameters 
cubeparameters; 


%Rename for  readability 
m_tot   = cube.m_tot;               % Mass of the cube
l       = cube.l_corner2cog;        % Length from corner to center of gravity
I2D     = cube.I_2D;                % Inertia 
kt      = motor.kt;                 % Motor constant

%Parameters for MPC
Q = diag([1 1]);                  % State weight
R = 1;                            % Input weight
N = 23;                            % Prediction horizion
x0 = [deg2rad(2) 0]';              % Initial state
i_con = 4 ;                        % Constring on input signal


%% Continous system matrices 

A = [0                        1                              
    m_tot*l*g/I2D             0];                              
    
B = [0 ; kt/I2D]; 

C = eye(2);

D =[];

inputnames ='current'; 
statenames = {'theta_c'  'omega_c'};

%-----Pack system 
sys_c = ss(A,B,C,D, 'Inputname',inputnames, 'Statename',statenames);

n = length(A);     % Number of states
[~,m] = size(B);   % Number of inputs

sys_d = c2d(sys_c, Ts.controller); % System discetization

%% Define the cost funtion on quadratic form

Hx              = kron(eye(N), Q);
[m_Hx , n_Hx]   = size(Hx); 

Hu              = kron(eye(N), R); 
[m_Hu, n_Hu]    = size(Hu); 

H = 2 * [Hx                        zeros(m_Hx, n_Hu ); 
         zeros(m_Hu, n_Hx)         Hu];
     
f = zeros(1,(n*N)+(m*N));

[L,p] = chol(H,'lower');
Linv = inv(L);
     
%% Define the constrain matrices

% Getting AA        
AA = [ sys_d.A ; zeros((n*N)-n,n)];
     
% Getting Aeq    
Btemp = kron(eye(N*m),-sys_d.B);
Atemp = [zeros(n,N*n) ; -kron(eye(N-1),sys_d.A) zeros((N-1)*n,n)] + eye(N*n);
Aeq =   [Atemp Btemp];
    
% Getting Ain 
Ain=[zeros(n*N,n*N) ,kron(eye(N),[1 ; -1])]; % Use empty matrices for the first case without actuator...

% Getting Bin
bin=[-i_con*ones(n*N,1)]; 


%% Create Struct
MPC = struct('Linv',Linv,...
             'f',f,...
             'Aeq',Aeq,...
             'AA',AA,...
             'Ain',Ain,...
             'bin',bin,...
             'iA0',false(size(bin)),...
             'n',n,...
             'N',N); 
         
         
%% Simulate 
options = optimset('Algorithm','interior-point-convex','Display','off');

xk = x0;
T=100;      % simulation time
 
yvec=[];
uvec=[];
for k=1:T
% 
%   cost     =  z^tHz + f'*z , f =0
%   Ain      : Constraints on states? 
%   Bin      : Constrains on insignal? 
%   Aeq, Beq : Determines state transition?? 

beq  = single(MPC.AA)*xk;

[z, ~, ~, ~] = myMPC(single(MPC.Linv), single(MPC.f'), single(MPC.Ain), single(MPC.bin),...
                         single(MPC.Aeq), single(beq), MPC.iA0);        % Solve MPC 

uk=double(z(n*N+1));                    %Extract the first insignal  
xk=sys_d.A*xk+sys_d.B*uk;       %Update time 
yvec=[yvec  sys_d.C*xk];        %Save outsignal 
uvec=[uvec; uk];                %Save insignal 
end
    
%% Plots 
close all; 
set(0,'defaulttextinterpreter','latex')

tvec=Ts.controller*(1:1:T);

%-------------------------Quadprog MPC
plot(tvec,yvec,'-',tvec,uvec','--'); grid
ylim([-4 2])
xlabel('Time [s]'); 
l = legend('Angle $\theta$ [rad]','Angular rate $\omega$ [rad/s]', 'Input $i$ [A]'); 
set(l,'Interpreter','latex')


%%
set(0,'defaulttextinterpreter','latex')
plot(angle.time,angle.data,'-',omega.time,omega.data,'-')
hold on; grid on;
plot(iref.time,iref.data,'--')
ylim([-4 2])
l = legend('Angle $\theta$ [rad]','Angular rate $\omega$ [rad/s]', 'Input $i$ [A]'); 
set(l,'Interpreter','latex')