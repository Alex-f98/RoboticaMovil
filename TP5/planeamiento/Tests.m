clc; close all; clear all;

%%TEST NEIGHBORS
MAP= zeros( 15,15);
          %  y x
test_point= [1,1];

figure()
imagesc(MAP);
axis([-1 17 -1 17])
%hold on;
%plot(test_point(1), test_point(2), '*r')
MAP(test_point(1), test_point(2))= 100;
grid('minor')% on

n = neighbors(test_point, [15,15]);

for i= 1:size(n,1)
  %       y       x
   MAP(n(i,2), n(i,1)) = 200;
end

imagesc(MAP)

%% TEST 