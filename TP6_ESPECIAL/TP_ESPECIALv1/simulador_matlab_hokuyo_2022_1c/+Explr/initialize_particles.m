function [particles, map] = initialize_particles(count, ranges, angles, max_range, map_size_x, map_size_y, map_resolution)
    % Returns a set of randomly initialized particles.

	mu = [map_size_x/2; map_size_y/2]./map_resolution;
	sigma = diag([map_size_x, map_size_y])./map_resolution;
    
    %Crear particulas
    particles = [mvnrnd(mu, sigma, count)'; unifrnd(-pi, pi, 1, count)];
	
    %Crear nuevo mapa por cada particula.
	map = occupancyMap.empty(count,0);
	
	for i = 1:count
		map(i) = occupancyMap(0.5*ones(map_size_x, map_size_y), map_resolution);
		move(map(i), [particles(1,i), particles(2,i)]-mu');
		insertRay(map(i), particles(:,i), lidarScan(ranges, angles), max_range);
	end

end

