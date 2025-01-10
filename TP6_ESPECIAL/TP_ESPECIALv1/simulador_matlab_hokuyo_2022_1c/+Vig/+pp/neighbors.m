function n = neighbors(cell, map_dimensions)

  n = [];

  pos_x = cell(2);
  pos_y = cell(1);
  size_x = map_dimensions(2);
  size_y = map_dimensions(1);
  
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
        n(end+1,:)= [y, x];
                
    end
  end
  
  
%   %%
%   
%   % mientras la pos_{x,y} tenga vecinos en todos sus lados.
%   if (1 < pos_x && pos_x < size_x) && (1 < pos_y && pos_y < size_y)
%       n= [ pos_x+1, pos_y; pos_x-1, pos_y;       % lado izquierdo y derecho.
%            pos_x, pos_y+1; pos_x, pos_y-1;       % lado arriba y abajo.
%            pos_x+1, pos_y+1; pos_x-1, pos_y-1;   % esquinas '/'.
%            pos_x-1, pos_y+1; pos_x+1, pos_y-1;   % esquinas '\'.
%           ];
%    % Casos borde:
%   %R: lado inferior.
%   elseif (1 < pos_x && pos_x < size_x)&&(pos_y == 1) 
%       n= [ pos_x+1, pos_y; pos_x-1, pos_y;       % lado izquierdo y derecho.
%            pos_x, pos_y+1;                       % lado arriba.
%            pos_x+1, pos_y+1; pos_x-1, pos_y+1;   % esquinas superiores.
%           ];
%   %R: lado superior.
%   elseif (1 < pos_x && pos_x < size_x)&&(pos_y == size_y)
%       disp("lado superior")
%       n= [ pos_x+1, pos_y; pos_x-1, pos_y;       % lado izquierdo y derecho.
%            pos_x, pos_y-1;                       % lado abajo.
%            pos_x-1, pos_y-1; pos_x+1, pos_y-1;   % esquinas inferiores.
%            %pos_x-1, pos_y+1; pos_x+1, pos_y+1;   % esquinas inferiores.
%           ];
%   %R: lado izquierdo
%   elseif (1 < pos_y && pos_y < size_y)&&(pos_x == 1) 
% %        disp("todo y pero x=1")
%       n= [ pos_x, pos_y+1; pos_x, pos_y-1;       % lado arriba y abajo.
%            pos_x+1, pos_y;                       % lado derecho.
%            pos_x+1, pos_y+1; pos_x+1, pos_y-1;   % esquinas.
%           ];
%   %R: lado derecho
%   elseif (1 < pos_y && pos_y < size_y)&&(pos_x == size_x) 
%   %    disp("todo y pero x=1")
%       n= [ pos_x, pos_y+1; pos_x, pos_y-1;       % lado arriba y abajo.
%            pos_x-1, pos_y;                       % lado izquierdo.
%            pos_x-1, pos_y-1; pos_x-1, pos_y+1;   % esquinas.
%           ];
%   %R: esquina inferior izquierda.
%   elseif (pos_x == 1)&&(pos_y == 1)
%       n= [ pos_x+1, pos_y+1; pos_x+1, pos_y; pos_x, pos_y+1;];
%   
%   %R: esquina inferior derecha.
%   elseif (pos_x == size_x)&&(pos_y == 1)
%       n= [ pos_x-1, pos_y+1; pos_x-1, pos_y; pos_x, pos_y+1;];
%   
%   %R: esquina superior derecha.
%   elseif (pos_x == size_x)&&(pos_y == size_y)
%       n= [ pos_x-1, pos_y-1; pos_x-1, pos_y; pos_x, pos_y-1;];
%       
%   %R: esquina superior izquierda.
%   elseif (pos_x == 1)&&(pos_y == size_y)
%       n= [ pos_x+1, pos_y-1; pos_x, pos_y-1; pos_x+1, pos_y;];
%   end
%   
%   n= [n(:,2), n(:,1)];
  % Return nx2 vector with the cell coordinates of the neighbors. 
  % Because planning_framework.m defines the cell positions as pos = [cell_y, cell_x],
  % make sure to return the neighbors as [n1_y, n1_x; n2_y, n2_x; ... ]

end