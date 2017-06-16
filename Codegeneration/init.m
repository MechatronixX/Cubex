% init.m

% Script for including path as lib, Wajung and UC3M for communication to
% the cube. Also open teh OnBoardMatlabExpo2 to run. Run this before
% compiling and uploading program to the cube.

addpath(genpath('../Libraries'))
addpath(genpath('../Codegeneration'))
addpath(genpath('../Simulation'))
addpath('C:\waijung15_04a/Addons/UC3M')
run('cubeparameters.m')