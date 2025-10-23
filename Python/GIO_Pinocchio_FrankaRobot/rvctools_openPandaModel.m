close all
clear all
clc

addpath('..\Downloads\rvctools\')

%% Import urdf model of Franka robot
robot = importrobot('franka_description\robots\frankaEmikaPanda.urdf');
show(robot)