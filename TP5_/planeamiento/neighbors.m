function n = neighbors(cell, map_dimensions)

  n = [];

  pos_x = cell(2);
  pos_y = cell(1);
  size_x = map_dimensions(2);
  size_y = map_dimensions(1);
  
  %%% YOUR CODE FOR CALCULATING THE NEIGHBORS OF A CELL GOES HERE
  
  % Return nx2 vector with the cell coordinates of the neighbors. 
  % Because planning_framework.m defines the cell positions as pos = [cell_y, cell_x],
  % make sure to return the neighbors as [n1_y, n1_x; n2_y, n2_x; ... ]
  
  for i =[-1, 0, 1]
    for j =[-1, 0, 1]
        
        x = pos_x + i;
        y = pos_y + j;
        
    %Si no cumple las condiciones no pertenece a un vecino
        if (x < 0) || (size_x <= x )
             continue;
        end
        
        if (y < 0) || (size_y <= y)
             continue;
        end
        
        if (i == 0) && (j == 0)
             continue;
        end
        n(end+1,:)= [x, y];
                
    end
  end

end