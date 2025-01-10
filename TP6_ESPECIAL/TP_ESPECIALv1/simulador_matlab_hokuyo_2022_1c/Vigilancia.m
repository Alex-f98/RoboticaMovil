function [v_cmd, w_cmd] = Vigilancia(idx, pose_, range, angles, map, test)
    %Introduction to Motion Planning Algorithms | Motion Planning with the
    %RRT Algorithm, Part 1:    https://www.youtube.com/watch?v=-fePRPyeKnc

    persistent path;
    %persistent vel_XYTheta;
    persistent mcl;
    persistent located;
    persistent exist_path;
    persistent estimatedPose;
    %persistent startPose;
    persistent goalPose;
    persistent idx_ref;
    persistent is_time;
    persistent is_orient;
    sampleTime = 0.1;
    puntoA = [1.5, 1.3]; %m      %puntoA= [3, 1]; %m
    puntoB = [4.3, 2.1]; %m      %puntoB= [1.1, 2.85]; %m
    if idx == 2
        is_time = false;
        located = false;
        exist_path = false;
        idx_ref = inf;
        goalPose = [puntoA , 0]; %world2grid(map, puntoB ); %[70 70 -pi/2];
    end

    scan = lidarScan(range, angles);
     
    v_cmd = 0;
    w_cmd = 0;

    %startPose = [puntoA , 0]; %world2grid(map, puntoA ); %[40 50 pi/2]; % [meters, meters, radians]


%Inicializar el objeto AMCL (Localiizacion Monte Carlo Aumentada)
   if idx == 2
       %%%%%%%%%%%%%%%%%%%%%%%___MCL___%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       mcl = monteCarloLocalization;
       mcl.UseLidarScan = true;
       mcl.GlobalLocalization = true; %false;
       %mcl.InitialPose = startPose;
       %mcl.InitialCovariance = eye(3)*0.5;
       mcl.UpdateThresholds = [0 0 0]; 
       
       sm = likelihoodFieldSensorModel;
       sm.SensorLimits = [0.06 5]; %m
       sm.NumBeams = 171;
       sm.MaxLikelihoodDistance = 5;
       sm.Map = map;
       
       mcl.SensorModel = sm;
       [isUpdated, estimatedPose, covariance] = mcl(pose_, scan);
       
       if isUpdated && test
           [particles, weights] = getParticles(mcl);
           figure(1)
           show(map)
           hold on
           scatter(estimatedPose(1), estimatedPose(2),...
               'Marker', 'o',...
               'MarkerFaceColor', 'g',...
               'MarkerEdgeColor', 'g')
           
           scatter(particles(:,1), particles(:,2),...
               'Marker', '.',...
               'MarkerFaceColor', 'r',...
               'MarkerEdgeColor', 'r')
           hold off
           %mcl.GlobalLocalization = false;
       end
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
   end
   %localization
   if (idx >= 2 && ~located)
       [isUpdated, estimatedPose, covariance] = mcl(pose_, scan);
       
       %Velocidades para que se localice inicialmente
              %oriento hacia la meta
       [v_cmd, w_cmd, is_orient] = orient(estimatedPose, goalPose, sampleTime);         %###############################
       v_cmd = 0.01;
       
%        dx = goalPose(1) - estimatedPose(1);
%        dy = goalPose(2) - estimatedPose(2);
%        w_cmd = 0.7*( atan2(dy, dx) - estimatedPose(3));
       
       %Saturador.
       [v_cmd, w_cmd] = sature(v_cmd, w_cmd);
       %disp("covarianza:" + num2str(covariance))
       
       if isUpdated && test
           [particles, weights] = getParticles(mcl);
           figure(5)
               show(map)
               hold on
               scatter(estimatedPose(1), estimatedPose(2),...
                   'Marker', 'o',...
                   'MarkerFaceColor', 'g',...
                   'MarkerEdgeColor', 'g')
               scatter(particles(:,1), particles(:,2),...
                   'Marker', '.',...
                   'MarkerFaceColor', 'r',...
                   'MarkerEdgeColor', 'r')
               hold off
           
           %mcl.GlobalLocalization = false;
       end
       
     %Si las diagoales de la covarianza es < 0.1 asumo que estoy localizado.  
       if diag(covariance) < 0.1
           disp("Localizado");
           located = true;
           startPose = estimatedPose;
           dx = goalPose(1) - estimatedPose(1);
           dy = goalPose(2) - estimatedPose(2);
           goalPose(3) = atan2(dy, dx);
           v_cmd= 0; w_cmd = 0;
           
       end
       
   end
 % ACA YA ESTOY LOCALIZADO: hacer ruta...  
     
          
   % Se ejecuta siempre que se necesite una nueva ruta: (start, goal, time)
   if located && ~exist_path
       %oriento hacia la meta
       [isUpdated, estimatedPose, covariance] = mcl(pose_, scan);
       [v_cmd, w_cmd, is_orient] = orient(estimatedPose, goalPose, sampleTime);         %###############################
       [v_cmd, w_cmd] = sature(v_cmd, w_cmd);
       %
       if is_orient
           path = codegenPathPlanner(map, estimatedPose, goalPose, test);
           %path = Vig.pp.path_planning(map, startPose, goalPose);

    %       EN PRINCIPIO VAMOS A IR A LA META SIN RESTRICCIONES DE TIEMPO
           [vel_XYTheta, path] = expectVel(path, sampleTime, 60);
           exist_path = true;
           idx_ref = idx;
            if test
               %----------------------------------------------------------------------
                figure(5)
                scatter(path(:,1),path(:,2),...
                    'Marker', 'o',...
                    'MarkerFaceColor', 'r',...
                    'MarkerEdgeColor', 'r')
               %----------------------------------------------------------------------
            end
           disp("ruta computada");
       end
   end

   if (idx_ref < idx) && (idx < ( idx_ref + length(path) - 1 )) && exist_path
       [isUpdated, estimatedPose, covariance] = mcl(pose_, scan);
       %[v_cmd, w_cmd] = ControltoGoal(path(idx,:)', pose_, vel_XYTheta(idx-1,:)');
       
       %Velocidad deseada.
       vel = (path(idx - idx_ref, :) - estimatedPose) ./ sampleTime;
       
       %Control de seguimiento de ruta.
       [v_cmd, w_cmd] = ControltoGoal(path(idx - idx_ref,:)', estimatedPose', vel');
       %[v_cmd, w_cmd] = ControltoGoal(path(idx,:)', estimatedPose, [0,0,0]);
       
       %Saturador.
       [v_cmd, w_cmd] = sature(v_cmd, w_cmd);
       
       startPose = estimatedPose;
       
%        if (diag(covariance) < 0.1) & (idx >= ( idx_ref + length(path) - 2 ))
%            startPose = estimatedPose;
%            v_cmd= 0; w_cmd = 0;
%        end
       
       %Debug:
       if test
           pause(0.2);
           fprintf("[v_cmd, w_cmd]:[%f, %f]\n",v_cmd, w_cmd)

              %-----------------------------------------------------------------------
               if isUpdated
                       %[particles,weights] = getParticles(mcl);
                       hold on
                       scatter(estimatedPose(1), estimatedPose(2),...
                           'Marker', 'o',...
                           'MarkerFaceColor', 'g',...
                           'MarkerEdgeColor', 'g')
                end
               %-----------------------------------------------------------------------
       end
   end
   
   %Gestiono las referencias desde el punto A hacia el puntoB.
   if (idx > ( idx_ref + length(path) - 1 ) ) && is_orient
       %esperar 3 segundos en la meta.
       if ~is_time
           [timer, is_time] = waitTime(3, sampleTime); %segundos
           disp("timer3s:" + num2str(timer));
       else
       
       %LLegue a la meta, pasaron 3 segundos, tengo que relcalcular la
       %trayectoria hacia el puntoB
           exist_path = false;
           %startPose = [puntoA , 0];
           goalPose = [puntoB , 0];   
       end
   end

    disp("runing: idx: " + num2str(idx));
    %disp("pose:" + "[" +num2str(path(idx,:))+ "]")
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%###################################                 ####################################################################
%###################################    FUNCIONES    ####################################################################
%###################################    AUXILIARES   ####################################################################
%###################################                 ####################################################################
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function path = codegenPathPlanner(mapData, startPose, goalPose, test)
% Copyright 2021 The MathWorks, Inc.
%Esta función actúa como un envoltorio para una llamada de planificación de ruta Híbrido A* estándar
%Acepta entradas estándar y devuelve una ruta sin colisiones como una matriz. 
%Dado que no puede usar un objeto identificador como entrada o salida de una función compatible con 
%la generación de código, cree el objeto del planificador dentro de la función.
    %inflate(map,0.17);
    % Create a state space object
    stateSpace = stateSpaceSE2;
   
    % Construct a state validator object using the statespace and map object
    validator = validatorOccupancyMap(stateSpace);
    
    %Cargar mapa de ocupacion.
    validator.Map = mapData;
    
    % Set the validation distance for the validator
    validator.ValidationDistance = 0.01*(1/mapData.Resolution); % meters
    
    % Assign the state validator object to the plannerHybridAStar object
    planner = plannerHybridAStar(validator,'MinTurningRadius', 0.7,'MotionPrimitiveLength',...
               0.2,'InterpolationDistance', 0.1,'ReverseCost', 7);
    
    % Compute a path for the given start and goal poses
    pathObj = plan(planner, startPose, goalPose);
    
    % Extract the path poses from the path object
    path = pathObj.States;
    
    %Graficar si estoy en la etapa de prueba test= true.
    if test
        figure(5)
        show(planner)
        title("ruta planeada en el espacio de grilla")
        
    %----------------------------------------------------------------------
    figure(5)
    hold on
    scatter(path(:,1),path(:,2),...
        'Marker', 'o',...
        'MarkerFaceColor', 'b',...
        'MarkerEdgeColor', 'b')
    %legend("Ruta generada A start","StartPose","Goal Pose"," Generated Path")
    %----------------------------------------------------------------------
    end
        
   % https://www.youtube.com/watch?v=-fePRPyeKnc
end

function [vel, Omega] = ControltoGoal(X_deseado, X_real, Xp_deseado)
%Controlador: segun el modelo diferencial.
% dx = vel.cos(theta)
% dy = vel.sen(theta)
% dtheta = Omega

% lo espresamos matricialmente:
%    Xp = J.Qp , donde:
%
%    Xp = [dx dy dtheta]'        Qp= [vel Omega]'
%
%        |cos(theta) 0 |
%    J = |sen(theta) 0 |
%        |    0      1 | 

%Entonces el error es:
% Xe =  X(deseado)  -  X(real)
% derivo= tal que Xe_p es la derivada del error
% Xe_p = Xp(deseado)  -  Xp(real)
% despejo el Xp(real)       ----> Xp(real) = Xp(deseado) - Xe_p  
% reemplazo:
% Xp = J.Qp   ---> Qp(real)= J^-1 . Xp(real)
% ################ Qp(real)= J^-1 . [ Xp(deseado) - Xe_p ]######(Xp= 0)????

%ESTABILIDAD: busco una funcion candidara de Lyapunov V()
% V(Xe) = Xe' Xe /2
% Vp= Xe'.Xe_p --->   #########Xe_p = -K.Xe###############
% Condicion de estabilidad Vp = -Xe'.K.Xe < 0, donde K es definida positiva (diagonal)
% J= [ cos(theta), 0]
%    [ sin(theta), 0]
%    [          0, 1]
 
    Xe =  X_deseado  -  X_real;
    %Xe_p = Xp_deseado  -  Xp_real;
    K = 0.5 .* diag([1 1 1]);
    %K = diag([1 1 1]);
    Xe_p = K*Xe;
    %Xp_deseado = 0; %velocidad deseado, por ahora es cero.
    pinvJ = @(theta)([ cos(theta)/(cos(theta)^2 + sin(theta)^2), sin(theta)/(cos(theta)^2 + sin(theta)^2), 0;
                                            0                  ,                  0,                       1]);
                                        
%     invJ = @(theta, d) (   [ cos(theta)/(2*cos(theta)^2 - 1) ,     sin(theta)/(2*sin(theta)^2 - 1)    ;
%                           sin(theta)/(d*(2*sin(theta)^2 - 1)), cos(theta)/(d*(2*cos(theta)^2 - 1)) ] );

                                        
    %Qp(real)= pinvJ * [ Xp(deseado) - Xe_p ]; -->X(deseado) = cte.
    Qp = pinvJ(X_real(3)) * ( Xp_deseado + Xe_p );
    %Qp = invJ(X_real(3), 0.01) .* ( Xp_deseado(1:2) + Xe_p(1:2) );
    
    %Qp= [vel Omega]
    vel = Qp(1);
    Omega= Qp(2);
end

function [v_cmd2, w_cmd2] = sature(v_cmd1, w_cmd1)
%     Comandos de velocidad (v,w) m/s
%     Rango permitido de velocidad lineal v (-0,5. . . 0,5) m/s
%     Rango recomendado de velocidad lineal v (-0,15. . . 0,15) m/s
%     Rango permitido de velocidad angular w (-4,25. . . 4,25) rad/s
%     Rango recomendado de velocidad angular w (-0,5. . . 0,5) rad/s
    vv = 0.15;
    ww = 0.15;
    v_cmd2 = v_cmd1;
    w_cmd2 = w_cmd1;
    
    if v_cmd1 > vv; v_cmd2 = vv; end
    if v_cmd1 < -vv; v_cmd2 = -vv; end
    
    if w_cmd1 > ww; w_cmd2 = ww; end
    if w_cmd1 < -ww; w_cmd2 = -ww; end
      
end

function [vel_XYTheta, Npath] = expectVel(path, sampleTime, time)
    %     Comandos de velocidad (v,w) m/s
    %     Rango permitido de velocidad lineal v (-0,5. . . 0,5) m/s
    %     Rango recomendado de velocidad lineal v (-0,15. . . 0,15) m/s
    %     Rango permitido de velocidad angular w (-4,25. . . 4,25) rad/s
    %     Rango recomendado de velocidad angular w (-0,5. . . 0,5) rad/s
    vv = 0.15;
    ww = 4.25;
    cant_muestras = time / 0.1;
    %obtengo las muestras necesarias
%     time1 = 0:sampleTime:(length(path)-1)*sampleTime;
%     time2 = 0:sampleTime/NS:(length(path))*sampleTime;
    time1 = 1:1:length(path);
    time2 = 1:(length(path)-1)/cant_muestras:length(path);
    
    Npath = interp1(time1, path, time2, 'spline');
    
    Xp_deseado = diff([Npath; Npath(end,:)]) ./sampleTime;
    
    J= @(theta)( [ cos(theta), 0; sin(theta), 0; 0, 1] );
    %pinvJ = @(theta)([ cos(theta)/(cos(theta)^2 + sin(theta)^2), sin(theta)/(cos(theta)^2 + sin(theta)^2), 0;
    %                                       0                  ,                  0,                       1]);
    [maxX, idxs] = max(Xp_deseado);
    Limit = J( Npath(idxs(3), 3) )*[vv ww]';
    
    if abs(Xp_deseado) > abs(Limit')
         vel_XYTheta = Limit .* (Xp_deseado ./ abs(maxX));
         %NSampleTime = vel_XYTheta(:, 1) ./ Npath(:, 1);
    else
       vel_XYTheta = Xp_deseado;
    end
end

function [timer, is_time] = waitTime(segundos, sampleTime)
%     sampleTime = 0.1 segundos.
%     segundos = 3 segundos.
%     is_time: indica si se llego a los 3 segundos.
%     timer: indica el tiempo que transcurrio.
    persistent count;
    if isempty(count)
        count = 0;
    end
    count = count + sampleTime;
    if count < segundos
        timer = count;
        is_time = false;
    else
        timer = count;
        is_time = true;
    end
        
end

function [v_cmd, w_cmd, is_orient] = orient(pose, goalPose, sampleTime)
    %Oriento al robot apuntando hacia la meta.
    dx = goalPose(1) - pose(1);
    dy = goalPose(2) - pose(2);
    kw = 0.1;
    theta_d = atan2(dy, dx);
    
    g_tol= (pi/4) ; % grados de tolerancia
    theta_err = theta_d - pose(3);
    if (theta_d - g_tol < pose(3) ) && (pose(3) < theta_d + g_tol)
        v_cmd = 0; w_cmd = 0;
        is_orient = true;
        disp("Robot orientado")
        %return
    else
        v_cmd = 0; 
        w_cmd = kw * theta_err / sampleTime;
        is_orient = false;
        disp("Orientando...")
    end
    
end
