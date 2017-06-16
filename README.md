# README #

This repository contains the file needed to model and control Cubex. Cubex is a Cubli based platform that can currently balance on it is corner. The work is part of a master thesis carried out at Chalmers University of Technology and Combine AB. 

For an example of corner balancing using this repository and Cubex see: http://mechatronicsblogger.blogspot.se/2017/05/cubex-first-corner-balancing-of-cubli.html

The thesis is to be released soon. 

### What is this repository for? ###

This is for resarchers interesting in modeling or controling the Cubex platform. It also features an implementation of fast appoximate gradient Model Predictive Control and a procedure to correct misaligned Inertial Measurement Units. 

### How do I get set up? ###
Quick summary of the folders. See each folder for further instructions

## IMU ## 
Find the misalignment of IMU's and a rotation matrix used to correct it. 

## Datasets ##
Sample IMU data. 

## Codegeneration ## 
Generate code using Simulink and the Waijung blockset for the STM32F4 onboard

## Simulation ## 
Simulation in Simulink using a non-linear state space. 

## Modeling ## 
Derivation and verification of the equations of motions. The "modeling_cube3d_kanes.nb" is the main file 

##  Libraries ## 
Files used both in modeling and codegeneration. 

### Contribution guidelines ###
Small oscillations are found during corner balancing, unclear why. It is probably related to the orientation estimation. 

Other algorithms for finding the center or gravity can be tried, see our thesis when it is released. 


### Who do I talk to? ###
Contact par.chalmers@gmail.com 