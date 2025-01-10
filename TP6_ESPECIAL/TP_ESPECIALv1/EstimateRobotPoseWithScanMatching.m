%% Estimate Robot Pose with Scan Matching

% Este ejemplo demuestra cómo hacer coincidir dos escaneos láser utilizando
% el algoritmo de Transformación de Distribuciones Normales (NDT) [1]. 
% El objetivo de la coincidencia de escaneo es encontrar la pose relativa 
% (o transformación) entre las dos posiciones del robot donde se tomaron los escaneos. 
% Los escaneos se pueden alinear según las formas de sus características superpuestas.

% Para estimar esta pose, NDT subdivide el escaneo láser en celdas 2D y a cada 
% celda se le asigna una distribución normal correspondiente. 
% La distribución representa la probabilidad de medir un punto en esa celda. 
% Una vez que se calcula la densidad de probabilidad, un método de optimización 
% encuentra la pose relativa entre el escaneo láser actual y el escaneo láser de referencia. 
% Para acelerar la convergencia del método, se puede proporcionar una suposición inicial de la pose. 
% Por lo general, la odometría del robot se utiliza para proporcionar la estimación inicial. 

% Si aplica la coincidencia de escaneo a una secuencia de escaneos, puede usarla 
% para recuperar un mapa aproximado del entorno que atraviesa el robot. 
% La coincidencia de escaneo también juega un papel crucial en otras aplicaciones, 
% como el seguimiento de posición y la localización y mapeo simultáneos (SLAM).
% 

%% Los datos del escaneo láser fueron recolectados por un robot móvil en un ambiente interior.
load lidarScans.mat

%% Plot Two Laser Scans    
%Elija dos escaneos láser para escanear coincidencias de lidarScans. 
%Deben compartir características comunes al estar juntos en la secuencia.

referenceScan = lidarScans(180);  %lidarScans(n) en el n-esimo tiempo tiene un ser de mediciones.
currentScan = lidarScans(202);

%Muestre los dos escaneos. Observe que hay compensaciones de traslación y rotación, 
% pero algunas características aún coinciden.

currScanCart = currentScan.Cartesian;
refScanCart = referenceScan.Cartesian;

figure
plot(refScanCart(:,1),refScanCart(:,2),'k.');
hold on
plot(currScanCart(:,1),currScanCart(:,2),'r.');
legend('Reference laser scan','Current laser scan','Location','NorthWest');

%% Run Scan Matching Algorithm and Display Transformed Scan
%Pase estos dos escaneos a la función matchScans. 
%MatchScans calcula la pose relativa del escaneo actual con respecto al escaneo de referencia.

transform = matchScans(referenceScan, currentScan);
%Muestre el escaneo de referencia junto con el escaneo láser actual transformado. 
%Si la coincidencia de escaneo fue exitosa, los dos escaneos deberían estar bien alineados.

%Para verificar visualmente que la pose relativa se calculó correctamente, 
%transforme el escaneo actual por la pose calculada usando transformScan. 
%Este escaneo láser transformado se puede utilizar para visualizar el resultado.
transScan = transformScan(referenceScan, transform);

figure
plot(currScanCart(:,1), currScanCart(:,2),'k.');
hold on
transScanCart = transScan.Cartesian;
plot(transScanCart(:,1),transScanCart(:,2),'r.');
legend('Reference laser scan','Transformed current laser scan','Location','NorthWest');

%% Build Occupancy Grid Map Using Iterative Scan Matching
close all; clc;

%Si aplica la coincidencia de escaneo a una secuencia de escaneos, 
%puede usarla para recuperar un mapa aproximado del entorno. 
%Utilice la clase occupancyMap para crear un mapa de cuadrícula de ocupación probabilística del entorno.

% Cree un objeto de cuadrícula de ocupación para un área de 15 metros por 15 metros. 
% Establezca el origen del mapa en [-7.5 -7.5].
map = occupancyMap(15,15,20); %map = occupancyMap(width,height,resolution)
map.GridLocationInWorld = [-7.5 -7.5];

%Asigne previamente una matriz para capturar el movimiento absoluto del robot. 
%Inicializa la primera pose como [0 0 0]. Todas las demás poses son relativas al primer escaneo medido.

%n = numel(A) returns the number of elements, n, in array A, equivalent to prod(size(A)).
numScans = numel(lidarScans);
initialPose = [0 0 0];
poseList = zeros(numScans,3);
poseList(1,:) = initialPose;
transform = initialPose;

%Cree un bucle para procesar los escaneos y mapear el área. 
%Los escaneos láser se procesan en pares. Defina el primer escaneo como 
%escaneo de referencia y el segundo escaneo como escaneo actual.
%Luego, los dos escaneos se pasan al algoritmo de coincidencia de escaneo y se calcula la pose relativa 
%entre los dos escaneos. La función "exampleHelperComposeTransform" se utiliza para calcular la pose absoluta 
%acumulativa del robot. Los datos de escaneo junto con la pose absoluta del robot 
%se pueden pasar a la función insertRay de la cuadrícula de ocupación.

% Recorra todos los escaneos y calcule las poses relativas entre ellos

for idx = 2:numScans
    % Procese los datos en pares.
    referenceScan = lidarScans(idx-1);
    currentScan = lidarScans(idx);
    
    % Ejecutar scan matching. Tenga en cuenta que los ángulos de escaneo permanecen iguales y no 
    % no tiene que ser recalculado. Para aumentar la precisión, establezca el máximo 
    % número de iteraciones hasta 500. Utilice la transformación de la última 
    % iteración como estimación inicial.
    [transform, stats] = matchScans(currentScan,referenceScan, ...
        'MaxIterations',500,'InitialPose',transform);
    
    %El |Score| en la estructura de las estadísticas es una buena indicación de la 
    % de calidad de la coincidencia de escaneo
    if stats.Score / currentScan.Count < 1.0
        disp(['Low scan match score for index ' num2str(idx) '. Score = ' num2str(stats.Score) '.']);
    end
    
    %Mantener la lista de poses de robot.
    absolutePose = exampleHelperComposeTransform_(poseList(idx-1,:),transform);
    poseList(idx,:) = absolutePose;
    
    %Integrar el escaneo láser actual en la ocupación probabilística
    %Grid
    insertRay(map, absolutePose, currentScan, 10);
    
    figure(1)
    show(map);
    title('Occupancy grid map built using scan matching results');
    
end

figure
show(map);
title('Occupancy grid map built using scan matching results');

%Plotear las poses absolutas del robot que fueron calculadas por el algoritmo de coincidencia de escaneo. 
%Esto muestra el camino que tomó el robot a través del mapa del entorno.

hold on
plot(poseList(:,1),poseList(:,2),'bo','DisplayName','Estimated robot position');
legend('show','Location','NorthWest')


function composedTransform = exampleHelperComposeTransform_(baseTransform, relativeTransform)
    %exampleHelperComposeTransform Compose two transforms
    %   The RELATIVETRANSFORM is added to the BASETRANSFORM and the composed
    %   transform is returned in COMPOSEDTRANSFORM.
    %   BASETRANSFORM is the transform from laser scan 1 to world and
    %   RELATIVETRANSFORM is the transform from laser scan 2 to laser scan
    %   1 (as returned by matchScans).

    % Concatenate the 4x4 homogeneous transform matrices for the base and
    % relative transforms.
    tform = pose2tform(baseTransform) * pose2tform(relativeTransform);      %POSE(k)= POSE(k-1).Ak2k+

    % Extract the translational vector
    trvec = tform2trvec(tform);                                             %[x,y,z] = tform2trvec(POSE(K))

    % Extract the yaw angle from the resulting transform
    eul = tform2eul(tform);                                                 %eulZYX = tform2eul(POSE(k))
    theta = eul(1);  % The default order for Euler angle rotations is 'ZYX'

    % Composed transform has structure [x y theta(z-axis)]
    composedTransform = [trvec(1:2) theta];
end
    
function tform = pose2tform(pose)
    %pose2tform Convert [x y theta] pose into homogeneous transform
    %   TFORM is returned as a 4x4 matrix.

    x = pose(1);
    y = pose(2);
    theta = pose(3);
    tform = trvec2tform([x y 0]) * eul2tform([theta 0 0]);
end

%% https://la.mathworks.com/help/nav/ug/estimate-robot-pose-with-scan-matching.html
