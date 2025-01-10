%% odometria EFK PREDICCION
clc; clear all; close all

syms 'xp' 'x' 'yp' 'y' 'thetap' 'theta' 'real'

% d_trans = sqrt( (xp - x)^2 + (yp -y)^2 );
% d_rot1 = atan2(yp-y, xp-x);
% d_rot2 = thetap - theta - d_rot1;

syms 'd_trans' 'd_rot1' 'd_rot2'
f_x = x + d_trans*cos(theta + d_rot1);
f_y = y + d_trans*sin(theta + d_rot1);
f_theta = theta + d_rot1 + d_rot2;

G= jacobian([f_x, f_y, f_theta], [x, y, theta])
h= jacobian([f_x, f_y, f_theta], [d_trans, d_rot1, d_rot2])

%% Odometria EKF CORRECCION.

syms 'r_med' 'mx' 'my' 'mu_x' 'mu_y' 'mu_theta' 'real'

h = [ sqrt( (mx - mu_x)^2 + (my - mu_y)^2 )]%; atan2( my - mu_y, mx - mu_x ) - mu_theta]

H = jacobian([h(1)], [mu_x, mu_y]);         %pag:185
H= simplify(H)



