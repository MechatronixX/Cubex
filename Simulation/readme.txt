File of interest:

- cubeparameter.m contain all the parameters of the cube. Running this script is needed before any 
  simulation or implementaion. 

- LQR_2d.m and LQR_3d.m calculate a infinte LQR controller for the edge balancing, respectively corner balancing. 
  This script saves files called either K_lqr.mat or K_lqr_3d.mat that is the numerical value of the K matrix. 

- run_simulation_2D.m and run_simulation_3D.m simulate edge balancing and corner balancing using Matlab.

- cube_2d_simulation_model.slx and cube_3d_simulation_model.slx is the Simulink models of the non-linear model
  for edge balancing and corner balancing. 