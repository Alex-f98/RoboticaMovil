function [v_cmd, w_cmd] = VelLidarSLAM(idx, ranges, angles, pose, sampleTime)

%persistent slamAlg;
persistent old_pose;

mapResolotion = 25; %20 celdas por metro, precisión de 100cm/20cm = 5cm
maxLidarRange = 5;

slamAlg = robotics.LidarSLAM(mapResolotion, maxLidarRange);
slamAlg.LoopClosureThreshold = 205;
slamAlg.LoopClosureSearchRadius = 8;

scans = lidarScan(ranges, angles);
if idx == 2
        old_pose = zeros(size(pose));
end
relPoseEst = relPos(pose, old_pose);

if idx < 11
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans, relPoseEst);
    if isScanAccepted
        fprintf('Added scan %d \n',idx-1);
    end
%     figure(1)
%     show(slamAlg);
%     title({'Map of the Environment','Pose Graph for Initial 10 Scans'});
elseif idx == 11

    %Reconstruya la escena trazando los escaneos y las poses rastreadas por slamAlg.
    figure(5)
        show(slamAlg);
        title({'Map of the Environment','Pose Graph for Intial 10 Scans'});
        firstTimeLCDetected = false;
end


if idx > 11 && idx < 100
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans);
    if ~isScanAccepted
        disp("no es aceptado")
    end
    

    if optimizationInfo.IsPerformed 
        figure(5)
            show(slamAlg, 'Poses', 'off', 'Parent', ax); hold on;
            slamAlg.PoseGraph.show('Parent', ax);
            firstTimeLCDetected = true;
            drawnow
    end
    
%     sampleTime=10;
%     [~,pose2] = scansAndPoses(slamAlg);
% [v_cmd, w_cmd] = pose2vel(pose2, sampleTime)
end

if idx == 100
    
    figure(5)
    show(slamAlg);
    title({'Final Built Map of the Environment', 'Trajectory of the Robot'});


    [scansSLAM,poses] = scansAndPoses(slamAlg);
    occMap = buildMap(scansSLAM, poses, mapResolotion, maxLidarRange);
    figure(7)
        show(occMap)
        title('Occupancy Map of Garage')

    disp("fin de simulacion")
end

[v_cmd, w_cmd] = Explorer(scans, maxLidarRange);

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

function [v_cmd, w_cmd] = Explorer(scans, rangMax)
%Debe determinar el siguiente paso a seguir para el mapeo con SLAM
% debe recorrer el mapa lo mas que se pueda pero cuando la incertidumbre
% sobrepase un umbral, es decir el error acumulado sea alto, debe de volver
% lo antes posible para cerrar un lazo y asi reducir incertesas gracias al
% scan matching
persistent v_cmd_;
persistent w_cmd_;

if isempty(v_cmd_)
    v_cmd_ = 0; 
    w_cmd_ = 0;
end

%Diametro total = 25cm.
    diametro = 25/100;
    offSet = 10/100;
    dw = 0.1;
    dv = 0.01;
    
    %v_perm = 0.5; v_perm_= -0.5;
    v_reco = 0.15; v_reco_ = -0.15;
    
    %w_perm = 4.25; w_perm_= -4.25;
    w_reco = 0.5; w_reco_ = -0.5;
%     
%     
%     scans.Ranges
%     scans.Angles
%     
    [minRang, idx] = min(scans.Ranges);
    
    %girar si el robot está muy cerca de algun obstaculo o pared.
    if minRang < (diametro+ offSet)
        %Girar en sentido contrario al minimo encontrado
        if idx < length(scans.Ranges)
            sgn= -1;
        else
            sgn= +1;
        end
        
        w_cmd_ = sgn * (w_cmd_ + dw);
        if w_cmd_ < w_reco_; w_cmd_ = w_reco_; end
        if w_cmd_ > w_reco; w_cmd_ = w_reco; end
        
        v_cmd_ = v_cmd_ - dv;
        if v_cmd_ <= dv; v_cmd_ = dv; end
    end
    
    %movernos linealmente...
    if  ( (diametro + offSet) < min(scans.Ranges) ) && ( min(scans.Ranges) < rangMax )
        v_cmd_ = v_cmd_ + dv;
        if v_cmd_ > v_reco; v_cmd_ = v_reco + dv; end
        
        w_cmd_ = 0;%w_cmd - 5*dw;
        if w_cmd_ > w_reco; w_cmd_ = w_reco + dw; end
        if w_cmd_ < w_reco_; w_cmd_ = w_reco_ - dw; end
    end
    
    
    v_cmd= v_cmd_;
    w_cmd= w_cmd_;
    
end
