function [mu, sigma] = correction_step(mu, sigma, z, l)
    % Updates the belief, i. e., mu and sigma, according to the sensor model
    %
    % The employed sensor model is range-only.
    %
    % mu: 3 x 1 vector representing the mean (x, y, theta) of the normal distribution
    % sigma: 3 x 3 covariance matrix of the normal distribution
    % z: structure containing the landmark observations, see
    %    read_data for the format
    % l: structure containing the landmark position and ids, see
    %    read_world for the format


    % Compute the expected range measurements.
    % This corresponds to the function h.
    expected_ranges = zeros(size(z, 2), 1); %z = data.timestep(t).sensor->[id,range,bearing]
    %En cada paso de tiempo vas a tener un H que va a tener dimension: cant. de mediciones x 2.
    for i = 1:size(z, 2)
        % Todo: Implement
        expected_ranges(i)= z(i).range;   
    end

    
    % Jacobian of h
    H = zeros(size(z, 2), 3);
    
    %Calculo algunas "constantes".
    mu_x = mu(1); mu_y= mu(2);
    %q = -sqrt(mu_x^2 + mu_y^2);
    
    % Measurements in vectorized form
    Z = zeros(size(z, 2), 1);
    for i = 1:size(z, 2)
        %H=[ (mu_x - mx)/((mu_x - mx)^2 + (mu_y - my)^2)^(1/2), (mu_y - my)/((mu_x - mx)^2 + (mu_y - my)^2)^(1/2)]
        %q = sqrt((l(idx).x - mu_x)^2 + (l(idx).y - mu_y)^2)
        idx = z(i).id;   
        q = norm([ (l(idx).x - mu_x), (l(idx).y - mu_y) ]);
        
        H(i,:) = [ (mu_x - l(idx).x )/q, (mu_y - l(idx).y )/q, 0];
        Z(i) = q;       %sqrt( (l(idx).x - mu_x)^2 + (l(idx).y - mu_y)^2 ); 
    end

    R = diag(repmat([0.5], size(z, 2), 1));

    K = sigma*H'/(H*sigma*H' + R);
    %   3x3*3xn / (nx3*3x3*3xn + nxn) == 3xn
    mu = mu + K*(expected_ranges - Z);
    sigma = ( eye(size(sigma)) - K*H )*sigma;
end
