%% Generate Code for Path Planning Using Hybrid A Star

%Este ejemplo muestra cómo realizar la generación de código para planificar 
%una ruta sin colisiones para un vehículo a través de un mapa utilizando el algoritmo Hybrid A*.
%Después de verificar el algoritmo en MATLAB®, 
%use el archivo MEX generado en el algoritmo para visualizar la ruta planificada

%% Verify Path Planning Algorithm in MATLAB
%Verifique el algoritmo de planificación de rutas en MATLAB antes de generar el código. 
clc; clear; close all;
%Cargue un mapa en el espacio de trabajo
mapData = load("exampleMaps.mat").simpleMap;

%Cree un objeto de espacio de estado.
stateSpace = stateSpaceSE2;

%Cree un mapa de ocupación binario.
binMap = binaryOccupancyMap(mapData);

%Construya un objeto de validación de estado utilizando el espacio de estado y el objeto de mapa.
stateValidator = validatorOccupancyMap(stateSpace,'Map',binMap);

%Establecer la distancia de validación para el validador
stateValidator.ValidationDistance = 0.01;

%Inicialice el objeto plannerHybridAStar con el objeto del validador de estado.
planner = plannerHybridAStar(stateValidator);

%Defina las poses de inicio y meta como vectores [x y theta]. 
%x e y especifican la posició|n en metros, y theta especifica el ángulo de orientación en radianes.
startPose = [5.5 5 pi/2];
goalPose = [22 4 0];

%Planifica un camino desde la pose inicial hasta la pose final
plan(planner,startPose,goalPose);

%Visualice la ruta usando la función show y oculte el árbol de expansión(tree).

show(planner,'Tree', 'off')
hold on


% %% Generate Code for Path Planning Algorithm
% 
% %Puede utilizar la función codegen (MATLAB Coder) o la aplicación MATLAB Coder (MATLAB Coder) para generar código
% %Para este ejemplo, genere un archivo MEX llamando a codegen en la línea de comandos de MATLAB. 
% %Especifique argumentos de entrada de muestra para cada entrada de la función 
% %mediante la opción -args y el argumento de entrada func_inputs
% codegen codegenPathPlanner -args {mapData, startPose, goalPose}
% 
% %% Verify Results Using Generated MEX Function
% % Planifique la ruta llamando a la versión MEX del algoritmo 
% % de planificación de ruta para la pose de inicio, la pose de destino y el mapa especificados.
% path = codegenPathPlanner_mex(mapData,startPose,goalPose);              %Genere código C/C++ a partir del código MATLAB
% 
% %Visualice la ruta calculada por la versión MEX del algoritmo de planificación de rutas.
% scatter(path(:,1),path(:,2),...
%         'Marker', 'o',...
%         'MarkerFaceColor', 'b',...
%         'MarkerEdgeColor', 'b')
% legend("MATLAB Generated Path","Start Pose","Goal Pose","MEX Generated Path")
% hold off
% 
% %% Check Performance of Generated Code
% time = timeit(@() codegenPathPlanner(mapData,startPose,goalPose));
% mexTime = timeit(@() codegenPathPlanner_mex(mapData,startPose,goalPose));
% time/mexTime;
% 
% %% Plan Path in New Map Using Generated MEX Function
% mapNew = load("exampleMaps.mat").simpleMap;
% startPoseNew = [10 8 pi];
% goalPoseNew = [5 22 0];
% pathNew = codegenPathPlanner_mex(mapNew,startPoseNew,goalPoseNew);
% 
% %Visualize the new path computed by the MEX function.
% show(binaryOccupancyMap(mapNew))
% hold on
% scatter(pathNew(:,1),pathNew(:,2),...
%         'Marker','o',...
%         'MarkerFaceColor',[0 0.4470 0.7410],...
%         'MarkerEdgeColor',[0 0.4470 0.7410])


%% Write Algorithm to Plan Path

%Cree una función, codegenPathPlanner, que use un objeto plannerHybridAStar 
%para planificar una ruta desde la posición de inicio hasta la posición de destino en el mapa

function path = codegenPathPlanner(mapData,startPose,goalPose)
% Copyright 2021 The MathWorks, Inc.
%Esta función actúa como un envoltorio para una llamada de planificación de ruta Híbrido A* estándar
%Acepta entradas estándar y devuelve una ruta sin colisiones como una matriz. 
%Dado que no puede usar un objeto identificador como entrada o salida de una función compatible con 
%la generación de código, cree el objeto del planificador dentro de la función.
    %#codegen
    % Create a state space object
    stateSpace = stateSpaceSE2;
   
    % Create a binary occupancy map
    binMap = binaryOccupancyMap(mapData);

    % Construct a state validator object using the statespace and map object
    validator = validatorOccupancyMap(stateSpace,'Map',binMap);
    
    % Set the validation distance for the validator
    validator.ValidationDistance = 0.01;
    
    % Assign the state validator object to the plannerHybridAStar object
    planner = plannerHybridAStar(validator);
    
    % Compute a path for the given start and goal poses
    pathObj = plan(planner, startPose, goalPose);
    
    % Extract the path poses from the path object
    path = pathObj.States;
end









