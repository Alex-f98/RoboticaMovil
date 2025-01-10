%function [v_cmd, w_cmd] = exploracion(idx, r, ranges, pose_)
function [v_cmd, w_cmd]= exploracion(idx, lidar, ranges, pose, v_cmd, w_cmd, n_particles, n_candidates, data_map, sampleTime, r, sigma_candidates)
% idx: indice de tiempo de simulacion.
% sampleTime: tiempo de muestreo.
% ej= r: DesiredRate: 10
%        DesiredPeriod: 0.1000
%        OverrunAction: 'slip'
%        TotalElapsedTime: 392.8105
%        LastPeriod: 0.0620
% pose_: POSE(idx,:) pose dada por la medicion de odometria actual.

persistent particles;
persistent old_particles;
persistent weights;
persistent maps;
persistent localized;
persistent is_obs;
persistent is_rot;
persistent old_pose;

if idx == 2
    localized = false;
    is_obs = false;
    is_rot = false;

    v_cmd = 0;
    w_cmd = 0;
    
    old_pose = [0;0;0];
end
%Tiempo de muestreo
sampleTime= r.DesiredPeriod;
%Dimensiones del mapa
map_size_x= data_map(1); map_size_y= data_map(2); map_resolution= data_map(3);

% si es la primer iteracion genero las particulas
	if(idx == 2)
		[particles, maps] = Explr.initialize_particles(n_particles, ranges, lidar.scanAngles,...
			lidar.maxRange, map_size_x, map_size_y, map_resolution);
	end
	
	% actualizo mapa con mediciones del lidar
	% insertRay(est_map, est_pose, lidarScan(ranges,lidar.scanAngles), lidar.maxRange);
	
	% estimo las poses de las particulas con el modelo de odometria
	% est_pose = est_pose + [v_cmd*cos(est_pose(3)); v_cmd*sin(est_pose(3)); w_cmd]*sampleTime;
	old_particles = particles;
    
    match_pose = matchScans(particles(1:2,:),particles(3,:),old_particles(1:2,:),old_particles(3,:));%,'SolverAlgorithm','fminunc');
    
%  	particles = particles + ...
%  		[v_cmd*cos(particles(3,:)); v_cmd*sin(particles(3,:)); repmat(w_cmd,1,size(particles,2))]*sampleTime;
%     % Compute new particle positions
%     odom = Explr.pose2odom(pose);
%     odom=odom .* 0.7;
%     particles = particles + [
%                              odom(2) .* cos( particles(3, :) + odom(1) );
%                              odom(2) .* sin( particles(3, :) + odom(1) );
%                              (odom(1) + odom(3)).*ones(size(particles(3,:)))
%                              ];
% 	
%  	particles = Explr.scan_match(particles, old_particles, ranges,...
%  		lidar.scanAngles, lidar.maxRange, [v_cmd, w_cmd], maps, sampleTime,...
% 		sigma_candidates, n_candidates);
% 	
	

    weights = Explr.measurement_model(ranges, lidar.scanAngles, lidar.maxRange, particles', maps);
    
	% detect NaN weights, if so they are discarded (weight set to zero)
	weights(isnan(weights)) = 0;
	
	% normalize the weights
    normalizer = sum(weights);
    weights = weights ./ normalizer;
	
	% update the maps
	if(idx > 2)
		for i=1:n_particles
			insertRay(maps(i), particles(:,i),...
				lidarScan(ranges, lidar.scanAngles), lidar.maxRange);
		end
	end
	
	% update the pose
	best_pose = particles*weights;
	
	% resample
	[particles, maps] = Explr.resample(particles, weights, maps);
	
	% uso lidar y mejoro poses
	% chequeo que no tenga obstaculos
	z_t = lidar.scanAngles;
	[min_ranges, min_idx] = min(ranges);
	min_angle = pi/(length(z_t)-1);
	% <)
	front_cone = (floor(length(z_t)/4)+1):(floor(3*length(z_t)/4));
	
	if(min(ranges(front_cone)) <= 0.25)
		is_obs = true;
		% localized = false; % para cuando agreguemos A*
		v_cmd = 0;
	else
		is_obs = false;
	end
	
	% IRFE (Initial Random Free Exploration)
	% si no esta localizado, ejecutar IRFE.
	if(localized == false)
		% si esta obstaculizado, que elija un sentido (horario
		% o antihorario) y rote hasta que se libere.
		if(is_obs == true)
			% si no esta rotando genera un w
			if(is_rot == false)
				% el w se genera segun los obstaculos que detecte...
				% si detecto que hay obstaculos en ambas direcciones,
				% giro aleatorio
				if(abs(z_t(min_idx)) <= min_angle)
					w_cmd = 0.75*(2*binornd(1,0.5)-1);
				% si detecto que hay obstaculos en angulos negativos,
				% entonces roto en sentido positivo
				elseif(z_t(min_idx) < -min_angle)
					w_cmd = 0.75;					
				elseif(z_t(min_idx) > min_angle)
					w_cmd = -0.75;					
				end
				is_rot = true;
				% si estaba rotando que siga rotando
			end
		% si no esta obstaculizado, que se mueva en direccion lineal
		else
			w_cmd = 0;
			is_rot = false;
			v_cmd = 0.25;
		end
	end

    % actualizar visualizacion
% 	if(idx > 2)
% 		delete(s)
% 	end
% 	
    %viz(pose(:,idx),ranges)
	hold on;
    %s = scatter(best_pose(1), best_pose(2), 'xk');
	%s.SizeData = 36; s.LineWidth = 2; hold off;
	
% 	for i = 1:n_particles
% 		figure(i+1);
% 		show(maps(i));
% 	end
	
	[~,indx] = max(weights);
	figure(2)
	show(maps(indx))

    waitfor(r);
    
end
