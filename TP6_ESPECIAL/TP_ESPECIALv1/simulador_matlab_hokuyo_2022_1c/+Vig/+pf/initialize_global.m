function [particles, w] = initialize_global(count, map)
    % Returns a set of randomly initialized particles.
%     particles = [
%         normrnd(0.0, 1, count, 1), ...
%         normrnd(0.0, 1, count, 1), ...
%         unifrnd(-pi, pi, count, 1)
%     ];

    %Tamaño del mapa de grilla
    x_size = map.GridSize(1);
	y_size = map.GridSize(2);
    
    %Region donde la grilla esta libre
    permisible_region= map.occupancyMatrix < map.FreeThreshold;
    
    %Obtengo los indices(1D) se esas grillas y seleccion 'count' grillas
    %con distribucion uniforme.
    idx= find(permisible_region(:) == 1 );
    %uniform_idx = datasample(find(permisible_region(:) == 1 ) ,count);
    uniform_idx = datasample(idx ,count);
    
    [row, col]= ind2sub([x_size y_size], uniform_idx);  %Ajustar!!!
    
    %POSES de particulas en el mundo.
    particles = [grid2world(map, [row, col]), unifrnd(-pi, pi, count, 1)];
    
    %Inicializo pesos.
    w= ones(count,1) ./ count;

end

