% Copyright 2018 The MathWorks, Inc.
%% Implement Simultaneous Localization And Mapping (SLAM) with Lidar Scans

%Este ejemplo demuestra cómo implementar el algoritmo de localización y 
%mapeo simultáneos (SLAM) en una serie recopilada de escaneos LIDAR mediante 
%la optimización de gráficos de pose. El objetivo de este ejemplo es construir 
%un mapa del entorno utilizando los escaneos LIDAR y recuperar la trayectoria del robot.

%Para construir el mapa del entorno, el algoritmo SLAM procesa de manera 
%incremental los escaneos lidar y crea un gráfico de pose que vincula estos escaneos. 
%El robot reconoce un lugar visitado previamente a través de la scan maching 
%y puede establecer uno o más cierres de bucle a lo largo de su ruta de movimiento. 
%El algoritmo SLAM utiliza la información de cierre de bucle para actualizar el mapa y ajustar 
%la trayectoria estimada del robot.
clc; clear; close all;
%% Load Laser Scan Data from File

%Cargue un conjunto de datos de muestreo reducido que consta de escaneos 
%láser recopilados de un robot móvil en un entorno interior. 
%El desplazamiento medio entre cada dos escaneos es de unos 0,6 metros.

%El archivo offlineSlamData.mat contiene la variable de escaneos, que 
%contiene todos los escaneos láser utilizados en este ejemplo.

load('offlineSlamData.mat');


%% Run SLAM Algorithm, Construct Optimized Map and Plot Trajectory of the Robot
%Cree un objeto lidarSLAM y configure la resolución del mapa y el rango máximo de lidar.
%Este ejemplo utiliza un robot Jackal™ de Clearpath Robotics™. El robot está equipado con un escáner láser SICK™ TiM-511 con un alcance máximo de 10 metros
%Establezca el rango máximo de lidar ligeramente más pequeño que el rango máximo de escaneo (8 m), 
%ya que las lecturas del láser son menos precisas cerca del rango máximo. 
%Establezca la resolución del mapa de cuadrícula en 20 celdas por metro, lo que da una precisión de 5 cm.

mapResolotion = 20; %20 celdas por metro, precisión de 100cm/20cm = 5cm
maxLidarRange = 8;  %8m como rango maximo

%Los siguientes parámetros de cierre de bucle se establecen empíricamente
%El uso de un umbral de cierre de bucle más alto ayuda a rechazar los falsos 
%positivos en el proceso de identificación de cierre de bucle.
%Sin embargo, tenga en cuenta que un match de puntuación alta puede seguir siendo un mal macht.
%Por ejemplo, es más probable que los escaneos recopilados en un entorno 
%que tiene características similares o repetidas produzcan falsos positivos.
%El uso de un radio de búsqueda de cierre de bucle más alto permite que el 
%algoritmo busque un rango más amplio del mapa alrededor de la estimación de pose actual para cierres de bucle.

slamAlg = robotics.LidarSLAM(mapResolotion, maxLidarRange);
slamAlg.LoopClosureThreshold = 205;
slamAlg.LoopClosureSearchRadius = 8;
%% Observe the Map Building Process with Initial 10 Scans

%Agregue escaneos de forma incremental al objeto slamAlg. 
%Los números de escaneo se imprimen si se agregan al mapa. 
%El objeto rechaza los escaneos si la distancia entre escaneos es demasiado pequeña. 
%Agregue los primeros 10 escaneos primero para probar su algoritmo

for i=1:10
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans{i});
    if isScanAccepted
        fprintf('Added scan %d \n',i);
    end
%     figure(1)
%     show(slamAlg);
%     title({'Map of the Environment','Pose Graph for Initial 10 Scans'});
end

%Reconstruya la escena trazando los escaneos y las poses rastreadas por slamAlg.
ax = newplot;
show(slamAlg, 'Parent', ax);
title(ax, {'Map of the Environment','Pose Graph for Intial 10 Scans'});

%% Observe the Effect of Loop Closures and the Optimization Process

%Continúe agregando escaneos en un bucle. Los cierres de bucle deben detectarse automáticamente a medida que el robot se mueve
%La optimización del gráfico de pose se realiza cada vez que se identifica un cierre de bucle.

%Trace los escaneos y poses cada vez que se identifique un cierre de bucle 
%y verifique los resultados visualmente. Este gráfico muestra escaneos superpuestos 
%y un gráfico de pose optimizado para el cierre del primer bucle.

firstTimeLCDetected = false; %indicador del primer cierre de lazo.

ax = newplot;
for i=10:length(scans)
%     addScan(slamObj,currScan):Agrega un escaneo lidar, currScan, al objeto lidar SLAM, slamObj. 
%     La función usa la coincidencia de escaneo para correlacionar este escaneo con el más reciente, 
%     luego lo agrega al gráfico de pose definido en slamObj. Si se acepta el escaneo, 
%     addScan detecta cierres de bucle y optimiza según la configuración en slamObj
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans{i});
    if ~isScanAccepted
        continue;
    end
    
%     figure(2)
%     show(slamAlg);
%     title({'Map of the Environment','Pose Graph for Initial 10 Scans'});
%     [~,poses] = scansAndPoses(slamAlg);
%     disp(num2str( poses(end-1,:) ));
    % visualizar el primer cierre de bucle detectado, si desea ver el 
    % del proceso de construcción del mapa completado, elimine la condición debajo
    if optimizationInfo.IsPerformed 
        show(slamAlg, 'Poses', 'off', 'Parent', ax); hold on;
        slamAlg.PoseGraph.show('Parent', ax);
        firstTimeLCDetected = true;
        drawnow
    end
    if i == 70
        disp("yoloyolo");
    end
%     sampleTime=10;
%     [~,pose2] = scansAndPoses(slamAlg);
% [v_cmd, w_cmd] = pose2vel(pose2, sampleTime)
end

title(ax, 'First loop closure');
figure
ax = newplot;
show(slamAlg, 'Parent', ax);
title(ax, {'Final Built Map of the Environment', 'Trajectory of the Robot'});


[scansSLAM,poses] = scansAndPoses(slamAlg);
occMap = buildMap(scansSLAM, poses, mapResolotion, maxLidarRange);
figure()
show(occMap)
title('Occupancy Map of Garage')

disp("fin de simulacion")

%[~,pose2] = scansAndPoses(slamAlg);
%[v_cmd, w_cmd] = pose2vel(pose2, sampleTime);


function [v,w]= pose2vel(pose, sampleTime)
   persistent old_pose;
   
   if ~isempty(old_pose)
       old_pose = zeros(size(pose));
   end
   
    v = norm( (pose(end, 1:2) - pose(end-1, 1:2)) ./ sampleTime );
    w = (pose(end, 3) - pose(end-1, 3)) ./ sampleTime;
    
    old_pose= pose;
end