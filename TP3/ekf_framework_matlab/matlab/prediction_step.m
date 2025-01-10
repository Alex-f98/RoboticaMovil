function [mu, sigma] = prediction_step(mu, sigma, u)
    % Updates the belief, i. e., mu and sigma, according to the motion model
    %
    % u: odometry reading (r1, t, r2)
    % mu: 3 x 1 vector representing the mean (x, y, theta) of the normal distribution
    % sigma: 3 x 3 covariance matrix of the normal distribution
    
    %Solo para enclarecer lo que se hace:
    d_rot1= u.r1; d_trans= u.t; d_rot2= u.r2;
    mu_x= mu(1); mu_y= mu(2); mu_theta= mu(3);
    
    %Jacobiano de g con respecto al control.   
    V = [ cos(d_rot1 + mu_theta), -d_trans*sin(d_rot1 + mu_theta), 0;
          sin(d_rot1 + mu_theta),  d_trans*cos(d_rot1 + mu_theta), 0;
                            0,                            1, 1];
    % Compute the noise-free motion. This corresponds to the function g, evaluated  
    % at the state mu.
    mu = [ mu_x + d_trans*cos(mu_theta + d_rot1);                          % g(mu, u{t-1})
           mu_y + d_trans*sin(mu_theta + d_rot1);
           mu_theta  +  d_rot1  +  d_rot2   ];

    % Compute the Jacobian of g with respect to the state
    G = [ 1, 0, -d_trans*sin(d_rot1 + mu_theta);
          0, 1,  d_trans*cos(d_rot1 + mu_theta);
          0, 0,                           1];

    % Motion noise
    Q = [0.2, 0, 0; 
        0, 0.2, 0; 
        0, 0, 0.02];
    % Predicción de la covarianza(V mapea Q al espacio de estado).
    sigma = G*sigma*G' + V*Q*V';
end
