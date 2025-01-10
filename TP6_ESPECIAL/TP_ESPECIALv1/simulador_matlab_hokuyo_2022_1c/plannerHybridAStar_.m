%plannerHybridAStar
clc; clear; close all;
load parkingLotCostVal.mat
load mapa_2022_1c.mat

Bmap = binaryOccupancyMap(map.occupancyMatrix > 0.5);

validator = validatorOccupancyMap;

validator.Map = Bmap;

planner = plannerHybridAStar(validator,'MinTurningRadius',4,'MotionPrimitiveLength',6);

puntoA = flip([1.5, 1.3]); %m      %puntoA= [3, 1]; %m
puntoB = flip([4.3, 2.1]); %m      %puntoB= [1.1, 2.85]; %m

startPose = [40, 50, pi/2];%world2grid(map, puntoA ); %[40 50 pi/2]; % [meters, meters, radians]
goalPose = [70, 70, -pi/2];%world2grid(map, puntoB ); %[40 55 -pi/2];
%para arreglar las cosas.
%goalPose(2)= goalPose(2)-30;
%%
refpath = plan(planner, startPose, goalPose);

show(planner)