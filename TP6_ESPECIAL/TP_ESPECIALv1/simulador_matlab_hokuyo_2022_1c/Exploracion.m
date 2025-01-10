function [v_cmd, w_cmd] = Exploracion(idx, ranges, angles, pose, sampleTime)
%% Implement Simultaneous Localization And Mapping (SLAM) with Lidar Scans
% https://www.youtube.com/watch?v=saVZtgPyyJQ&t=15s

%Este ejemplo demuestra cómo implementar el algoritmo de localización y 
%mapeo simultáneos (SLAM) en una serie recopilada de escaneos LIDAR mediante 
%la optimización de gráficos de pose.

%Para construir el mapa del entorno, el algoritmo SLAM procesa de manera 
%incremental los escaneos lidar y crea un gráfico de pose que vincula estos escaneos. 
%El robot reconoce un lugar visitado previamente a través de la scan maching 
% puede establecer uno o más cierres de bucle a lo largo de su ruta de movimiento. 
%El algoritmo SLAM utiliza la información de cierre de bucle para actualizar el mapa y ajustar 
%la trayectoria estimada del robot.

persistent old_pose;
persistent slamAlg;


scans = lidarScan(ranges,angles);
%lidarScan:= Ranges; Angles; Cartesian; Count

mapResolotion = 25; %celdas por metro, precisión de 100cm/25cm = 4cm
maxLidarRange = 5;  %5m como rango maximo

%Los siguientes parámetros de cierre de bucle se establecen empíricamente
%El uso de un umbral de cierre de bucle más alto ayuda a rechazar los falsos 
%positivos en el proceso de identificación de cierre de bucle.
%Sin embargo, tenga en cuenta que un match de puntuación alta puede seguir siendo un mal macht.
%Por ejemplo, es más probable que los escaneos recopilados en un entorno 
%que tiene características similares o repetidas produzcan falsos positivos.
%El uso de un radio de búsqueda de cierre de bucle más alto permite que el 
%algoritmo busque un rango más amplio del mapa alrededor de la estimación de pose actual para cierres de bucle.


if idx == 2
    slamAlg = robotics.LidarSLAM(mapResolotion, maxLidarRange);
    slamAlg.LoopClosureThreshold = 360;
    slamAlg.LoopClosureSearchRadius = 1;
end

%Agrega escaneos de forma incremental al objeto slamAlg. 

% Los cierres de bucle deben detectarse automáticamente a medida que el robot se mueve
%La optimización del gráfico de pose se realiza cada vez que se identifica un cierre de bucle.

%Este gráfico muestra escaneos superpuestos 
% firstTimeLCDetected = false; %indicador del primer cierre de lazo.

    if idx == 2
        old_pose = zeros(size(pose));
    end
    relPoseEst = relPos(pose, old_pose);
    
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans, relPoseEst);
%   addScan(slamObj,currScan):Agrega un escaneo lidar, currScan, al objeto lidar SLAM, slamObj. 
%   La función usa la coincidencia de escaneo para correlacionar este escaneo con el más reciente, 
%   luego lo agrega al gráfico de pose definido en slamObj. Si se acepta el escaneo, 
%   addScan detecta cierres de bucle y optimiza según la configuración en slamObj
    if isScanAccepted
        fprintf('Added scan %d \n',idx);
    else
         %continue;
         disp("Scann: No aceptado");
         v_cmd = -0.1;
         w_cmd = 0.1;
         return;
    end
    
    
    % visualizar el primer cierre de bucle detectado, si desea ver el 
    % del proceso de construcción del mapa completado, elimine la condición debajo
     if optimizationInfo.IsPerformed %&& ~firstTimeLCDetected
            figure(3)
            show(slamAlg, 'Poses', 'on'); 
            hold on;
            slamAlg.PoseGraph.show();
            firstTimeLCDetected = true;
        drawnow
    end

% 
%  title(ax, 'First loop closure');
%  figure(2)
%  ax = newplot;
%  show(slamAlg, 'Parent', ax);
%  title(ax, {'Final Built Map of the Environment', 'Trajectory of the Robot'});


[scansSLAM, poses] = scansAndPoses(slamAlg);
% figure(2)
% show(slamAlg)

occMap = buildMap(scansSLAM, poses, mapResolotion, maxLidarRange);

figure(1)
show(occMap, 'world')
title('Occupancy Map LAR')

[v_cmd, w_cmd] = ExplorerLop(scans, maxLidarRange);

%[v_cmd, w_cmd] = pose2vel(poses, sampleTime);

old_pose = pose;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%###################################                 ####################################################################
%###################################    FUNCIONES    ####################################################################
%###################################    AUXILIARES   ####################################################################
%###################################                 ####################################################################
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [v,w]= pose2vel(pose, sampleTime)
   persistent old_poses;
   disp("pose: " + num2str(pose));
   if isempty(old_poses)
       old_poses = zeros(size(pose));
   end
   
    v = norm( (pose(end, 1:2) - old_poses(end, 1:2)) ./ sampleTime );
    w = (pose(end, 3) - old_poses(end, 3)) ./ sampleTime;
    
    old_poses= pose;
end

function relPoseEst= relPos(currPos, refPos)
    
    %relPoseEst= zeros(size(refPos));

    ref= trvec2tform([refPos(1:2); 0]');
    curr= trvec2tform([currPos(1:2); 0]');
    
    ref(1:3,1:3)= rotz(refPos(3) * (180/pi));
    curr(1:3,1:3)= rotz(currPos(3) * (180/pi));
    
    %POSE(k)= POSE(k-1).Ak2k+
    rel= inv(curr) * ref;
    
    %relPoseEst(1:2)   = rel(1:2,4);
    eul = tform2eul(rel);                                                 %eulZYX = tform2eul(POSE(k)) [rad]
    relPoseEst= [rel(1:2,4); eul(1)] ;  % The default order for Euler angle rotations is 'ZYX'
    
end

function [v_cmd, w_cmd] = Explorer(scans_, rangMax)
%Debe determinar el siguiente paso a seguir para el mapeo con SLAM
% debe recorrer el mapa lo mas que se pueda pero cuando la incertidumbre
% sobrepase un umbral, es decir el error acumulado sea alto, debe de volver
% lo antes posible para cerrar un lazo y asi reducir incertesas gracias al
% scan matching
persistent v_cmd_;
persistent w_cmd_;
persistent count;

scans = lidarScan(scans_.Ranges(43:128), scans_.Angles(43:128));                                                       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if isempty(v_cmd_)
    v_cmd_ = 0; 
    w_cmd_ = 0;
    count = 0;
end

%Diametro total = 25cm.
    diametro = 25/100;
    offSet = 10/100;
    dw = 0.2;
    dv = 0.01;
    
    v_perm = 0.5; v_perm_= -0.5;
    v_reco = 0.15; v_reco_ = -0.15;
    
    w_perm = 4.25; w_perm_= -4.25;
    w_reco = 0.5; w_reco_ = -0.5;
    
    % Aca decido mis umbrales de velocidad.
    ww = w_reco; ww_ = w_reco_;
    %vv = w_reco; vv_ = v_reco_;
    vv = v_reco; vv_ = v_reco_;
%     
%     
%     scans.Ranges
%     scans.Angles
%     
    [minRang, idx] = min(scans.Ranges);
    
    %girar si el robot está muy cerca de algun obstaculo o pared.
    if minRang < (diametro + offSet)
        disp("girar: minRang < diametro + offSet");
        
        D = (diametro + offSet);
        k = (D - minRang) / D;
        
        %Girar en sentido contrario al minimo encontrado
        if idx < length(scans.Ranges)/2
            sgn= +1;
        else
            sgn= -1;
        end
        
        if count == 5
            %w_cmd_ = sgn * (w_cmd_ + dw);  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            w_cmd_ = sgn * k * w_perm;
            disp("w_cmd: " + num2str(w_cmd_))

            if w_cmd_ < ww_; w_cmd_ = ww_; end
            if w_cmd_ > ww; w_cmd_ = ww; end

            v_cmd_ = v_cmd_; %- dv;
            if v_cmd_ <= dv; v_cmd_ = dv; end
            count = 0;
        end
        count = count + 1;
        
    elseif  ( (diametro + offSet) < minRang) && ( minRang < rangMax )
        disp(" Lineal: diametro + offSet < minRang < rangMax  ")
        v_cmd_ = v_cmd_ + dv;
        if v_cmd_ > vv; v_cmd_ = vv + dv; end
        
        w_cmd_ = 0;%w_cmd - 5*dw;
        if w_cmd_ > ww; w_cmd_ = ww + dw; end
        if w_cmd_ < ww_; w_cmd_ = ww_ - dw; end
    end
    
    %Condicion extrema:
    if minRang < (diametro + offSet/2)
        disp("retro: minRang < (diametro + offSet/2) ")
        v_cmd_ = v_reco_;
        w_cmd_ = -ww;
    end
    
    
    
    v_cmd= v_cmd_;
    w_cmd= w_cmd_;
    
end
function [v_cmd, w_cmd] = ExplorerLop(scans_, rangMax)
    persistent old_idx1
    persistent old_idx2
    
    if isempty(old_idx1)
        old_idx1= 85;
        old_idx2= 86;
    end
scans = lidarScan(scans_.Ranges(43:128), scans_.Angles(43:128));  
    
%Diametro total = 25cm.
    diametro = 25/100;
    offSet = 10/100;
    dw = 0.2;
    dv = 0.01;
    
    v_perm = 0.5; v_perm_= -0.5;
    v_reco = 0.15; v_reco_ = -0.15;
    
    w_perm = 4.25; w_perm_= -4.25;
    w_reco = 0.5; w_reco_ = -0.5;
    
    v_cmd = v_reco;
    w_cmd = w_reco;
    
    [minRang, idx] = min(scans.Ranges);
    [MaxRang, idxM] = max(scans.Ranges);
    
    if (rangMax + 0.1) < (MaxRang && MaxRang <= rangMax)
        if idxM < length(scans.Ranges)/2
            sgn= -1;
        else
            sgn= +1;
        end
        
        %v_cmd = sgn*v_cmd;
        w_cmd = sgn*w_cmd;
    end
    
    %girar si el robot está muy cerca de algun obstaculo o pared.
    if minRang < (diametro + offSet)
        if idx < length(scans.Ranges)/2
            sgn= +1;
        else
            sgn= -1;
        end
        
        %v_cmd = sgn*v_cmd;
        w_cmd = sgn*w_cmd;
    end
    %estoy en una esquina.
    if ~(old_idx1 < length(scans.Ranges)/2) && ~(old_idx1 < length(scans.Ranges)/2) && (idx < length(scans.Ranges)/2) 
        
        if idxM < length(scans.Ranges)/2
            sgn= -1;
        else
            sgn= +1;
        end
        
        w_cmd = sgn*w_reco;
        v_cmd = 0;
    end
    
    old_idx2 = old_idx1;
    old_idx1= idx;
end


%% https://www.youtube.com/watch?v=saVZtgPyyJQ&t=1s
%% 
