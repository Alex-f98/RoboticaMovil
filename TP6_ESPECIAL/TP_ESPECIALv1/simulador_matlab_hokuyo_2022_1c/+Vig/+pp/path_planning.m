function [trayectory] = path_planning(mapa, start, goal)
map= mapa.occupancyMatrix;

% visualizes the map. Note: 
% the y-axis is inverted, so 0,0 is top-left, 
figure(1)
imagesc(map); %imshow(map); %
title('Map');
hold on;

% Obtener altura y ancho del mapa.
hw= mapa.GridSize;  %size(map);
h= hw(1); w= hw(2);

% valores de costo para cada celda, Inicializado en infinito
costs = ones(h,w)*inf;

% Costos estimado a la meta.
heuristics = zeros(h,w);

% cells that have been visited
closed_list = zeros(h,w);

% Estas matrices almacenan implícitamente la ruta al contener la posición x e y 
% del nodo anterior, respectivamente. Seguir estos comenzando en el objetivo
% hasta que se alcanza -1 devuelve la ruta calculada, vea en la parte inferior

previous_x = zeros(h,w)-1;
previous_y = zeros(h,w)-1;

% start and goal position (y,x)
%start = [34,23];
%goal = [16,41]; %[25, 23];
start = world2grid(mapa, start);
goal = world2grid(mapa, goal);

%plot start and goal cell
plot(start(2), start(1), 'r.');
plot(goal(2), goal(1), 'g.');
%pause(1); %pause for visualization

%Empezar a buscar desde start. 
parent= start;
costs(start(1),start(2)) = 0;
%costs(start(2),start(1)) = 0;

%itero hasta que 'goal' sea encontrada.
while (parent(1)~=goal(1) || parent(2)~=goal(2))

  %generar máscara para asignar costos infinitos para celdas ya visitadas
  closed_mask = closed_list;
  closed_mask( closed_mask == 1 )= Inf; 
  
  %Encontrar las candidatas para la expansión (open list/frontier)
  open_list = costs + closed_mask + heuristics;

  %comprobar si existe una entrada no infinita en la lista abierta (list is not empty)
  if min(open_list(:)) == Inf
    disp('no valid path found');
    break
  end

  %encontrar la celda con el costo mínimo en 'open_list'
  [y,x] = find(open_list == min(open_list(:)));
  parent_y = y(1);
  parent_x = x(1);
  parent = [parent_y, parent_x];
  
  %poner 'parent' en 'closed_list'
  closed_list(parent_y, parent_x) = 1;
  plot(parent_x, parent_y, ' y.' );
  
  %for visualization: Plot start again
  if(parent(1) == start(1) && parent(2) == start(2))
    plot(start(2), start(1), 'r.');
  end
  
  %Obtener 'neighbors' de 'parent'
  n = Vig.pp.neighbors(parent, [h, w]);
  
  for i=1:size(n,1) %para cada vecino(niño)
    child_y = n(i, 1);
    child_x = n(i, 2);
    child = [child_y, child_x];
    
    %Calcular el costo de llegar a la celda
    cost_val = costs(parent_y, parent_x) + Vig.pp.edge_cost(parent, child, map);
  
    %Exercise 2: estimar los costos restantes desde la celda hasta la meta(goal)
    heuristic_val = Vig.pp.heuristic(child, goal);

    %Actualizar el costo de la celda
    if cost_val < costs(child_y, child_x)
      costs(child_y,child_x) = cost_val;
      heuristics(child_y, child_x) = heuristic_val;
        
      %safe child's parent
      previous_x(child_y, child_x) = parent_x;
      previous_y(child_y, child_x) = parent_y;
    end
  end
  pause(0.05); %pause for visualization
end

% visualization: from the goal to the start,
% draw the path as blue dots
parent = [goal(1), goal(2)];
distance2 = 0;

%guardará a child para cada iteracion
trayectory=[];

while previous_x(parent(1), parent(2))>=0
  plot(parent(2), parent(1), 'm*');
  
  %for visualization: Plot goal again
  if(parent(1) == goal(1) && parent(2) == goal(2))
    plot(goal(2), goal(1), 'g.');
  end
  
  child_y = previous_y(parent(1), parent(2));
  child_x = previous_x(parent(1), parent(2));
  child = [child_y, child_x];
  distance2 = distance2 + norm(parent - child);
  parent = child;
  %ACTUALIZA EL TAMAÑO Y EL CONTENIDO DE 'trayectory' (es mas lento, CAMBIAR)
  trayectory(end+1,:)= child;
  
  pause(0.05); %pause for visualization
end

%Paso la turta de grillas a coordenadas del mundo.
trayectory = grid2world(mapa, trayectory);

disp 'done'
disp 'path cost: ', disp(costs(goal(1),goal(2)));
disp 'path length: ', disp(distance2);
disp 'number of nodes visited: ', disp(sum(closed_list(:)));

end