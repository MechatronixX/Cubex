function [MPC_3d, fMPC_3d, sys_d] = MPC_Parameters_3D(cube, motor, Ts, wheel)
    %% MPC parameters for 3D case. 
    % Script that calculate the parameters for a Model Predictive controller
    % and returns the values in a struct variable 
    % Argument: cube, motor, Ts

    %   Rename for  readability 
    mc      = cube.m_tot; 
    mw      = wheel.m;
    r       = cube.r;
    Ic      = cube.Ic; 
    Iw0     = wheel.Iw0; 
    kt      = motor.kt; %DEBUG: Seems that current become too large

    theta0 = atan(sqrt(2));
    g0 = 9.81; 

    s_para = 1; % need extra work

    %   Parameters for MPC
    Q = diag([20 20 20 .1 .1 .1]);   %Penalties on states, we care mostly about the angle 
    R = .1*eye(3);   %Voltage is our only input
    N = 30;                             % Prediction horizion 35
    i_con = 4*kt;                          % Constring on input signal

    % Scaling parameter that Gros said could be a problem to Fast MPC
    % Scale down the insignal closer to the state values

    i_con = i_con/s_para;   % Need to check out!

%% Continous system matrices 

    %A = [0,0,(-1).*2.^(1/2).*g0.*Ic.^(-1).*(mc+2.*mw).*r.*cot(theta0),0,0, ...
    %  0;0,2.^(-1/2).*g0.*Ic.^(-1).*(mc+2.*mw).*r.*(2.^(1/2).*cos(theta0) ...
    %  +2.*sin(theta0)),0,0,0,0;0,0,2.^(1/2).*g0.*Ic.^(-1).*(mc+2.*mw).* ...
    % r.*csc(theta0),0,0,0];
       A = [0,0,(-1).*2.^(1/2).*g0.*(mc+2.*mw).*r.*(Ic+2.*Iw0+(3.*mc+5.*mw).* ...
      r.^2).^(-1).*cot(theta0),0,0,0;0,2.^(-1/2).*g0.*(mc+2.*mw).*r.*( ...
      Ic+2.*Iw0+(3.*mc+5.*mw).*r.^2).^(-1).*(2.^(1/2).*cos(theta0)+2.* ...
      sin(theta0)),0,0,0,0;0,0,2.^(1/2).*g0.*(mc+2.*mw).*r.*(Ic+2.*Iw0+( ...
      3.*mc+5.*mw).*r.^2).^(-1).*csc(theta0),0,0,0];

    %This a only models the highest derivatives, augment to include lower
    %derivatives

    A = [ zeros(3,3), eye(3)  ; A];

    B = [2.^(-1/2).*(Ic+2.*Iw0+(3.*mc+5.*mw).*r.^2).^(-1).*(Ic+2.*(Iw0+ ...
      mw.*r.^2)).^(-1).*(Ic+2.*(Iw0+(mc+2.*mw).*r.^2)).*csc(theta0),2.^( ...
      -1/2).*(Ic+2.*Iw0+(3.*mc+5.*mw).*r.^2).^(-1).*(Ic+2.*(Iw0+mw.* ...
      r.^2)).^(-1).*(Ic+2.*(Iw0+(mc+2.*mw).*r.^2)).*csc(theta0),2.^(1/2) ...
      .*(mc+mw).*r.^2.*(Ic+2.*Iw0+(3.*mc+5.*mw).*r.^2).^(-1).*(Ic+2.*( ...
      Iw0+mw.*r.^2)).^(-1).*csc(theta0);2.^(-1/2).*(Ic+2.*Iw0+(3.*mc+5.* ...
      mw).*r.^2).^(-1),(-1).*2.^(-1/2).*(Ic+2.*Iw0+(3.*mc+5.*mw).*r.^2) ...
      .^(-1),0;(1/2).*(Ic+2.*Iw0+(3.*mc+5.*mw).*r.^2).^(-1).*(Ic+2.*( ...
      Iw0+mw.*r.^2)).^(-1).*(2.*(mc+mw).*r.^2+(-1).*2.^(1/2).*(Ic+2.*( ...
      Iw0+(mc+2.*mw).*r.^2)).*cot(theta0)),(1/2).*(Ic+2.*Iw0+(3.*mc+5.* ...
      mw).*r.^2).^(-1).*(Ic+2.*(Iw0+mw.*r.^2)).^(-1).*(2.*(mc+mw).*r.^2+ ...
      (-1).*2.^(1/2).*(Ic+2.*(Iw0+(mc+2.*mw).*r.^2)).*cot(theta0)),(Ic+ ...
      2.*Iw0+(3.*mc+5.*mw).*r.^2).^(-1).*(Ic+2.*(Iw0+mw.*r.^2)).^(-1).*( ...
      Ic+2.*Iw0+mc.*r.^2+3.*mw.*r.^2+(-1).*2.^(1/2).*(mc+mw).*r.^2.*cot( ...
      theta0))];

    B = [zeros(3,3); B];

    %We measure all
    C = eye(6); 

    inputnames ={'T1','T2','T3}'}; 
    statenames = {'phi', 'theta', 'psi','phidot' , 'thetadot', 'psidot'};


    sys_c = ss(A,B,C,[], 'Inputname',inputnames, 'Statename',statenames);
    Ts = Ts.controller;  
    sys_d = c2d(sys_c, Ts);
    n = length(A);
    m = size(B,2);
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
    bin=-i_con*ones(n*N,1); 

    %% Create Struct
    MPC_3d = struct('Linv',Linv,...
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

    u_lower = -i_con*ones(m,1);                      % Inequality constrain on input
    x_lower = -100*ones(n,1);                    % Set an arbitrary large constrain state so it not interfere

    z_lower = [u_lower ; x_lower];
    z_upper = -1 * [u_lower ; x_lower];
    for i = 2 : N
        z_lower =  [z_lower ; [u_lower ; x_lower]];
        z_upper =  [z_upper ; -1 * [u_lower ; x_lower]];
    end

    %% Rename report 
    % Using spliting 1 from report
    R = chol(Aeq*MPC_3d.iH*Aeq','lower');   
    M  = 0;%length(R)-90;             % For full banded matrix P -> set m = length(R)
    L = 3.5;
    P  = approx_preconditioner(R, M, MPC_3d.iH, Aeq);
    %% Struct for FastMPC

    fMPC_3d = struct('dd',single(AA),...
                  'miHDtPt',single(-MPC_3d.iH*Aeq'*P'),...
                  'LPD',single((1/L)*P*Aeq),...
                  'LP',single((1/L)*P),...
                  'inCo',single([z_lower z_upper]),...
                  'N',single(N),...
                  'nx',single(MPC_3d.n),...
                  'mu',single(MPC_3d.m),...
                  'L',single(L),...
                  'D',single(Aeq),...
                  'P',single(P),...
                  'M',M,...
                  's_para',single(s_para));
end