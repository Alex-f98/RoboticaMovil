function cost = edge_cost(parent, child, map)

  %cost = 0;
  Pm = 0.5;
  k= 10;
  %          y       x
  %child= [n(i,1), n(i,2)]
  prob_m= map(child(1), child(2));
  
  if prob_m >= Pm
      cost= inf;
  else
       %distancia[w] = distancia[vértice]+peso (vértice, w)
      cost= norm(parent - child) + k*prob_m;
  end
end