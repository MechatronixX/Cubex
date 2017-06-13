% init.m

% Script for including path as lib, Wajung and UC3M for communication to
% the cube. It is needed to download the programs to the cube. If not Waijung 
% or UC3M is installed on computer, it is not possible to transfer any program
% to the cube.

% For it to work Waijung and U3CM needs to be installed and also the path
% in Matlab needs to be located at either Codegeneration, Simulation or Libraries

addpath(genpath('../Libraries'))
addpath(genpath('../Codegeneration'))
addpath(genpath('../Simulation'))
addpath('C:\waijung15_04a/Addons/UC3M') 