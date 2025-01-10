function makePlot(X,P,filename)
% plot initial situation
  plot(X(1,:), X(2,:), 'ob');
  %xlim([-1, 11])
  %ylim([-6, 4])
  axis('square')
  hold on
  plot(P(1,:), P(2,:), 'or');

  legend('X', 'P at original location', ...
         'location', 'southwest');
  % draw data association
  hold on
  for j=1:size(P,2)
    line([P(1,j) X(1,j)],[P(2,j) X(2,j)]);
  end

  % save plot to file
  filename = sprintf(['../plots/' filename]);
  print(filename, '-dpng');
  hold off

end

