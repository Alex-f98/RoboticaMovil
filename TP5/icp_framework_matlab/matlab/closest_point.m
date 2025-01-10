%does closest-point matching between the point sets X and P.
%Input:  X, P.
%Output: P_matched, which you get by reordering the elements in P so
%        that they match the elements in X.
%
%Output: P_matched, que obtiene al reordenar los elementos en P para 
%        que coincidan con los elementos en X
function P_matched=closest_point(X,Pp)

    P= Pp;        %<--- para no modificar directamente el vector de entrada
    P_matched = zeros(size(P));

   for i= 1:size(X, 2) % para cada punto en X.
%     
      [p_min, p_idx]= min ( vecnorm ( P - X(:, i) ) ); %ocupa mas memoria en t_ejec.
      
      %Reordeno.
      P_matched(:,i)= P(:, p_idx);
      
      %redimensiono P para no tener mas de dos puntos macheados
      if (p_idx ~= size(P,2)) && (p_idx ~= 1)   % si: idx/=1 y idx/=16
          P= [P(:, 1:p_idx-1) , P(:, p_idx+1:end)];
      elseif p_idx == size(P,2)                 % si: idx=16
          P= P(:, 1:end-1);
      else                                      % si: idx=16
          P= P(:, 2:end);
      end
      
  end

end

