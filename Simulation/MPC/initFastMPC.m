% init for running the nonlinear model using Fast MPC
cubeparameters; 
[MPC, fMPC, sys_d] = MPC_Parameters(cube, motor ,Ts);