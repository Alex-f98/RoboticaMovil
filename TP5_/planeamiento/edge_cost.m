function cost = edge_cost(parent, child, map)

  cost = 0;
 
  %%% YOUR CODE FOR CALCULATING THE COST FROM VERTEX parent TO VERTEX child GOES HERE
  Pm = 0.5;
  k= 10;
  %          y       x
  %child= [n(i,1), n(i,2)]
  prob_m= map(child(2), child(1));
  
  
  if prob_m >= Pm
      cost= inf;
  else
       %distancia[w] = distancia[vértice]+peso (vértice, w)
      cost= norm(parent - child) + k*prob_m;
%       if(child == [39, 12])
%           cost=0;
%       end
  end
  
end