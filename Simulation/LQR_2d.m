%% Calculating LQR feedback parameters for the linearized 2d edge balancing problem

clear all; 

%Load the cube parameters 
cubeparameters; 

%% Continous system matrices 


%------------Model with current reference as insignal 

tau_i = motor.tau_cl; % Time constant i_ref --> i

A = [0                                                  1                               0 ;
    cube.m_tot*cube.l_corner2cog/cube.I_2D*g            0                               motor.kt/cube.I_2D;
    0                                                   0                               -1/tau_i];  
B = [0; 0; 1/tau_i]; 

C = [1 0 0]; 

D =[];

inputnames ='i_ref'; 
statenames = {'theta_c'  'omega_c' 'i' };

% %--------Model with voltage as insignal 
% A = [0                                          1                           0                        0;
%     cube.m_tot*cube.l_corner2cog/cube.I_2D*g    0                           0                        motor.kt/cube.I_2D;
%     0                                           0                           0                       -motor.kt/wheel.Iy;
%     0                                           -motor.kw/motor.L           motor.kw/motor.L        -motor.R/motor.L];
% 
% B = [0 ; 0; 0;  1/motor.L]; 
% 
% C = [1 0 0 0]; 
% % C  = [1 0 0 0; 
% %       0 1 0 0;
% %       0 0 1 0;
% %       0 0 0 0];
%   
% D=0; 
% inputnames ='Vin'; 
% statenames = {'theta_c'  'omega_c'  'omega_m' 'i' };

% %Model with torque as input
% A = [0                                                      1; 
%      (cube.m_tot)*(cube.l_corner2cog)*g/(cube.I_2D)         0]; 
%  
% B = [0 ; 2/cube.I_2D]; 
% 
% C =[1 0]; 
% 
% inputnames ='Torque'; 
% statenames = {'theta_c'  'omega_c' };


%-----Pack system 
sys_c = ss(A,B,C,[], 'Inputname',inputnames, 'Statename',statenames); 

Nx = length(A); 

%% System discetization
Ts = 0.01;  %Sampling time of choice 
sys_d = c2d(sys_c, Ts);

%% Reachability 

%-----Cont. time rechability
Co_cont = ctrb(sys_c); 
disp(['Continous time controllability matrix rank = ', num2str(rank(Co_cont))      ] );


%---Discrete reachability 
Co_disc = ctrb(sys_d); 

disp(['Discrete time reachability matrix rank = ', num2str(rank(Co_disc))      ] );


%% LQR 
Nx = length(A); 

if(Nx == 4)
        disp('Four state model: [Theta_c , omega_c, omega_1 , i_1 ]')
        Qx = diag([10000 0.1 0.1 0.1]);   %Penalties on states, we care mostly about the angle 
        Ru = 1;                       %Voltage is our only input
        x0= [pi/4 ; 0 ; 0;0]
elseif (Nx == 2)
        disp('Using two state model excluding motor model x = [Theta_c, omegac ]'); 
        Qx = diag([100 1]); 
        Ru =100; 
        x0= [deg2rad(10) ; 0];     
elseif (Nx == 3)
    disp('Three state model: [Theta_c , omega_c, i]')
    
      Qx = diag([0 0 1]);    
      Ru =1;                 %Insignal is a current reference for the motor
      
      x0= [deg2rad(6) ; 0 ;0 ];  
end

[K_lqr,~,~] = lqr(sys_d,Qx,Ru) 

eigenvalues = abs(eig(sys_d.A-sys_d.B*K_lqr))


%% Simulation
%Simulate the discretized closed loop system

%Override x0, using two state model now  
x0 = [degtorad(6) ; 0]; 

%Forcing the LQR to behave like a PD controller with setpoint = 0
% K_lqr(1) = 20;
% K_lqr(2) = 10; 
% K_lqr(3) = 0; 

%K_lqr = K_lqr *0.2; 

stopTime = 10; 

init = struct('theta',x0(1) ); 

sim('cube_2d_simulation_model'); 



%% Plots 
%plot(t, x(:,1));


close all; 
set(0,'defaulttextinterpreter','latex')

%plot(t, rad2deg(y(:,1)) );
t = simTime.data(:); 

yyaxis left
plot(t, rad2deg(cube_states.cube_angle.data(:) ), 'k'); 
hold on; 
yyaxis right 
plot(t, rad2deg(cube_states.cube_angular_velocity.data(:) ), 'r'); 

l = legend('Angle(Degrees)','Angular velocity (rad/s)'); 
set(l,'Interpreter','Latex'); 
xlabel('Time[s]');

figure; 

current = cube_states.motorStates.current.data(:); 

plot(t, current, 'b'); 
hold on; 
plot(iref.time, iref.data(:),'--b'); 
plot(iref.time, ones(size(iref.time))*motor.Imax, 'k--'); 
plot(iref.time, -ones(size(iref.time))*motor.Imax, 'k--'); 
%plot(t, 
l = legend('Current', 'Current reference', 'Allowable current'); 
set(l,'Interpreter','Latex'); 
xlabel('Time[s]');

title('Response to initial conditions using LQR control')



% Acl = [(sys_d.A-sys_d.B*K_lqr)];
% Bcl = [sys_d.B];
% %Bcl = [0 ;0 0;0]; 
% Ccl = [sys_d.C];
% Dcl = [];
% 
% %Convert into DISCRETE state space 
% sys_cl = ss(Acl,Bcl,Ccl,Dcl, Ts); 
% 
% t = 0:Ts:5;
% %r =0.2*ones(size(t));
% 
% %Zero insignal 
% u=zeros(size(t)); 
% 
% [y,t,x]=lsim(sys_cl,u,t, x0);

%[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
%set(get(AX(1),'Ylabel'),'String','cart position (m)')
%set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')







