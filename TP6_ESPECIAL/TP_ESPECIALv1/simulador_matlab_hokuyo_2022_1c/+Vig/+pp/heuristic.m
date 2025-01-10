function heur = heuristic(cell, goal)
  %1,2,5,10
  h = 1;

  heur= h*norm(cell - goal);
end
