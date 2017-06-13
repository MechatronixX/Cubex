CubeX project 2017-06 
Master thesis project

David Wall, Love Palm

Chalmers University of Technology
Combine AB

Program needed:
Matlab/Simulink
Mathematica 
Waijung blockset
UC3M

--------- MAPS --------------------

------ Codegeneration --------
Containing code that can be implemented in the cube. Some additional programs need to be installed to transfer
the Simulink code to the STM32F4 Discovery board. For more information about that see 'GettingStarted.txt'.
Script that is of most intreste in the map is 'LQR corner'. Other interessting script can be 'LQR edge'. 
There is also some test program to test for example 'I2C communication' and 'Motors'. There is an 'init.m' files 
including all the maps to the path in MATLAB and also the additional programs.  

------ Data/Datasets -----
Matlab data from the cube under running.

--------- IMU ------------------
Find the misaligment of the IMUs mounted on the cube. Collecting data from the cube and run the script
'findIMUpos' finds a rotation matrix for aligning the sensor frame with the body frame. For more information 
about the script, see the script 'findIMUpos'.

------ Libraries -------------
Containing libararies needed in simulation and code implementation. For estimate the orientation of the cube we
use Magdwick filter and additional script for converting quaternions to euler, which can be find in this map.
The 'Libararies' maps contain script and more that reuses in many scripts. 

----- Modeling -----
Uses Mathematica for modeling the equation of motion of the cube. This maps conatin the modeling part and 
linearization that can be further implemented in Matlab/Simulink for simulation and implmentation in the cube.

---- Simulation -----
Containg all the simulation of the model using LQR controller and MPC.
- cubeparameter.m contain all the parameters of the cube. Running this script is needed before any 
  simulation or implementaion. 

- LQR_2d.m and LQR_3d.m calculate a infinte LQR controller for the edge balancing, respectively corner balancing. 
  This script saves files called either K_lqr or K_lqr_3d that is the numerical value of the K matrix. 

- run_simulation_2D.m and run_simulation_3D.m simulate edge balancing and corner balancing using Matlab.

- cube_2d_simulation_model.slx and cube_3d_simulation_model.slx is the Simulink models of the non-linear model
  for edge balancing and corner balancing. 

MPC folder contating Fast MPC simulation of the edge balancing and corner balancing.

The listed script above is the most of interested script for simulate the model and further work. 




 