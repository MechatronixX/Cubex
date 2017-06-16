### Simulation ###
Simulation in 3D for corner balancing or 2D for edge balancing. Different controllers can be tried 


## Quick start ## 
Run either "run_simulation_2D.m" or "run_simulation_3D.m" to run a simulation and plot of result. 


## Files ## 
- cubeparameter.m contains all the parameters of the cube. Running this script is needed before any 
  simulation or implementaion. The parameters are stored in Matlab structs.  

- cube_2d_simulation_model.slx and cube_3d_simulation_model.slx is the Simulink models of the non-linear model
  for edge balancing and corner balancing. 

- LQR_2d.m and LQR_3d.m calculate a infinte horizon LQR gain matrix for the edge balancing and corner balancing respectively. This script saves the gain matrix in the files K_lqr.mat or K_lqr_3d.mat. 

- run_simulation_2D.m and run_simulation_3D.m simulate edge balancing and corner balancing using Matlab.

## Subfolders ## 
The folder MPC is a simulation of Cubex using Fast Proximate Gradient MPC. 