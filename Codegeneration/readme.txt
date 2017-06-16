Program for CubeX.

-------------------------------------------------------------
TO RUN OR DOWNLOAD SIMULINK MODELS TO THE CUBE, WAIJUNG and U3CM IS NEEDED TO BE INSTALLED
FOR MORE INFO SEE: 'GetstartedProgrammingCubeX' and 'Cubli notes' in Documentation folder. 
--------------------------------------------------------------

Some scribts in 'Simulation' and 'Libraries' folder is used for preinitial Simulink models under Model Properties. This is used for get the parameters
needed to matlab Workspace before download program. 

Script XXX.slx is used for programing the cube
Script with XXX_READ.slx is used for read values from the cube using USB communication.

To download a program to the cube it is nessery that the Matlab current folder is in the program folder. 
Ex. To download the LQR corner program to the cube, that is located at ...\Git\cubex\Codegeneration\LQR corner,
Matlab current path needs to point to that map.

'init.m' is a script including paths to Matlab/Simulink for the preintital scripts
For it to work Waijung and U3CM needs to be installed and also the path in Matlab needs to be located at
either Codegeneration, Simulation or Libraries

LQR corner 	- Program for balancing on the corner
LQR Edge   	- Program for balancing on the edge using LQR
MPC function 	- Program for balancing on the edge using 'Fast MPC'
Test Programs 	- Small program for individual part of the cube
I2C		- Test I2C commuication and read values from IMUs.