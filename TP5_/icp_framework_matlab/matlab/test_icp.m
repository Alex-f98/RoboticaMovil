clear
hold off
more off
% draw data association
bDraw = true;
% generating the data
X = [0, 1;
    1, 1;
    2, 1;
    3, 1;
    4, 1;
    5, 1;
    6, 1;
    7, 1;
    8, 1;
    9, 1;
    9, 0;
    9, -1;
    9, -2;
    9, -3;
    9, -4;
    9, -5]';

P = X + 0.1*randn(size(X,1), size(X,2));
P(1,:) = P(1,:)+1;
P(2,:) = P(2,:)+1;

theta1 =  5/360*2*pi;
theta2 = 110/360*2*pi;
rot1 = [cos(theta1), -sin(theta1); sin(theta1), cos(theta1)];
rot2 = [cos(theta2), -sin(theta2); sin(theta2), cos(theta2)];

% two sets with known correspondences
P1 = rot1*P;
P2 = rot2*P;

% two sets with unknown correspondences
P3 = P1(:,randperm(size(P,2)));
P4 = P2(:,randperm(size(P,2)));

% choose a set for testing, CHANGE accordingly!
P = P1;

P_original = P

if bDraw
  makePlot(X,P,'icp0000init0.png');
end


for i=1:15
%  closest-point matching 
  P = closest_point(X,P)


if bDraw && i==1
  makePlot(X,P,'icp0000init1.png')
end


  plot(X(1,:), X(2,:), 'ob');
  %xlim([-1, 11])
  %ylim([-6, 4])
  axis('square')
  hold on
  plot(P_original(1,:), P_original(2,:), 'or');
  
  
  % calculation of rotation and translation via ICP
  [R_est, t_est] = icp(X,P);
  P = R_est*P+t_est*ones(1,size(P,2));

  % visualization of the transformed point set P_transformed (magenta)
  plot(P(1,:), P(2,:), 'om');
  
  legend('X', 'P at original location', 'P transformed by ICP', ...
         'location', 'southwest');
if bDraw
  % draw data association
  hold on
  for j=1:size(P,2)
    line([P(1,j) X(1,j)],[P(2,j) X(2,j)]);
  end
end

  % save plot to file
  filename = sprintf('../plots/icp%03d.png', i+1);
  print(filename, '-dpng');
  hold off
end

