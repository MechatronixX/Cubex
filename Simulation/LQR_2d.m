%% Calculating LQR feedback parameters for the linearized 2d edge balancing problem


%Load the cube parameters 
cubeparameters; 

%% Continous system matrices 
A = [0                                          1                           0                        0;
    cube.m_tot*cube.l_corner2cog/cube.I_edge    0                           0                        0;
    0                                           0                           0                        motor.kt/wheel.Iy;
    0                                           0                           -motor.kw/motor.L        -motor.R/motor.L];

B = [0 ; 0; -1/motor.L]; 

C  = [1 0 0 0; 
      0 1 0 0;
      0 0 1 0;
      0 0 0 0];
  
D=0; 
  
sys_c = ss(A,B,C,D); 

%% System discetization
Ts = 0.01;  %Sampling time of choice 








