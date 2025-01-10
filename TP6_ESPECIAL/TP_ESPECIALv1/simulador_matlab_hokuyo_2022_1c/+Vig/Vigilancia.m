function [v_cmd, w_cmd]= Vigilancia(idx,dd, map, odom_, range, rangMax)
    %range: 171 puntos inicialmente. entre 0 y 5m:rangMax
    %odom: pose(:, idx) la pose estimada actual.
    
    persistent old_odom;
    if isempty(old_odom)
        old_odom = zeros(size(odom_));
    end
    %Calculo la verdadera odometria
    odom = odom_ - old_odom;
    
    %% Calcular la ruta.
    
    % si existe una ruta:true, si no existe una ruta:false
    exist_path = false; 
    
    puntoA= [1.5, 1.3]; %m      %puntoA= [3, 1]; %m
    puntoB= [4.3, 2.1]; %m      %puntoB= [1.1, 2.85]; %m

    % generar velocidades para este timestep idx.
        %A= [3, 1] %m
    if ~exist_path             %Genero trayectoria al inicio o cuando
        start= odom_(1:2,1)';                 %REVISAR CUAL SERA LA POSE
        goal= puntoA;
        trayectory = pp.path_planning(map, start, goal);
        exist_path= true;
    end
    %Inicializo las particulas
    cant_particulas= 500;
    persistent particle;
    persistent weight;
    if isempty(particle) %si las particulas y los pesos no fueron inicializados:
           [particle, weight]= pf.initialize_global(cant_particulas, map);
    end
    %[new_pose]= particle_filter(dd, v_cmd, w_cmd, particles, weight);
    
    
 
        
end