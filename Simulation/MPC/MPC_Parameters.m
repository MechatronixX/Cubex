function [MPC, fMPC, sys_d] = MPC_Parameters(cube, motor, Ts)
    %% MPC parameters for 2D case. 
    % Script that calculate the parameters for a Model Predictive controller
    % and returns the values in a struct variable 
    % Argument: cube, motor, Ts
    
    %   Rename for  readability 
    m_tot   = cube.m_tot;               % Mass of the cube
    l       = cube.l_corner2cog;        % Length from corner to center of gravity
    I2D     = cube.I_2D;                % Inertia 
    kt      = motor.kt;                 % Motor constant
    
    g       = 9.81;                     % Gravity

    %   Parameters for MPC
    Q = diag([200 10]);                 % State weight 200 10
    R = 1;                              % Input weight
    N = 35;                             % Prediction horizion 35
    i_con = 4;                          % Constring on input signal

    %% Continous system matrices 

    A = [0                        1                              
        m_tot*l*g/I2D             0];                              

    B = [0 ; kt/I2D]; 

    C = eye(2);

    D =[];


    %-----Pack system 
    sys_c = ss(A,B,C,D);

    n = length(A);     % Number of states
    [~,m] = size(B);   % Number of inputs

    sys_d = c2d(sys_c, Ts.controller); % System discetization
    
    %sys_d.A = [1 1 ; 0 1];% DEBUG!!!
    %sys_d.B = [0 ; 1]

    %% Define the cost funtion on quadratic form

    H = 2 * kron(eye(N),[R zeros(m,n) ; zeros(n,m) Q]);

    f = zeros(1,N*(n+m));

    L = chol(H,'lower');
    Linv = inv(L);

    %% Define the constrain matrices

    % Getting AA        
    AA = [ -sys_d.A ; zeros((n*N)-n,n)];


    % Getting equality constrain matrix Aeq
    Aeq=[sys_d.B -eye(n)];
    for i = 1 : N-1
        Aeq(end+1:end+n,end-n+1:end+m+n) = [sys_d.A sys_d.B -eye(n)];
    end


    % Getting inequality constrain matrix Ain 
    Ain_upper=[eye(m) zeros(m,n)];
    for i = 1 : N-1
       Ain_upper(end+1:end+m,end+1:end+n+m) =[eye(m) zeros(m,n)]; 
    end
    Ain_lower=[-eye(m) zeros(m,n)];
    for i = 1 : N-1
       Ain_lower(end+1:end+m,end+1:end+n+m) =[-eye(m) zeros(m,n)]; 
    end
    Ain = [Ain_upper ; Ain_lower];



    % Getting Bin
    bin=[-i_con*ones(n*N,1)]; 


    %% Create Struct
    MPC = struct('Linv',Linv,...
                 'H',H,...
                 'iH', inv(H),...
                 'f',f,...
                 'Aeq',Aeq,...
                 'AA',AA,...
                 'Ain',Ain,...
                 'bin',bin,...
                 'iA0',false(size(bin)),...
                 'n',n,...
                 'm',m,...
                 'N',N); 
             
    %% Inequality constrains for FastMPC

    u_lower = - i_con;                      % Inequality constrain on input
    x_lower = [-3 ; -20];                    % Set an arbitrary large constrain state so it not interfere

    z_lower = [u_lower ; x_lower];
    z_upper = -1 * [u_lower ; x_lower];
    for i = 2 : N
        z_lower =  [z_lower ; [u_lower ; x_lower]];
        z_upper =  [z_upper ; -1 * [u_lower ; x_lower]];
    end

    %% Rename report 
    % Using spliting 1 from report
    R = chol(Aeq*MPC.iH*Aeq','lower');   
    m = length(R);              % For full banded matrix P -> set m = length(R)
    L = 1;
    [P, L]  = approx_preconditioner(R, m, L);
    %% Struct for FastMPC

    fMPC = struct('dd',AA,...
                  'miHDtPt',-MPC.iH*Aeq'*P',...
                  'LPD',(1/L)*P*Aeq,...
                  'LP',(1/L)*P,...
                  'inCo',[z_lower z_upper],...
                  'N',N,...
                  'nx',MPC.n,...
                  'L',L,...
                  'D',Aeq,...
                  'P',P);
              
              
end