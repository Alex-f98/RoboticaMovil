function weight = measurement_model(z, x, l)
    % Computes the observation likelihood of all particles.
    %
    % The employed sensor model is range only.
    %
    % z: set of landmark observations. Each observation contains the id of the landmark observed in z(i).id and the measured range in z(i).range.
    % x: set of current particles
    % l: map of the environment composed of all landmarks
    
    sigma = [0.2];
    weight = ones(size(x, 1), 1);

    if size(z, 2) == 0
        return
    end
 %voy a tener size(z,2) mediciones de landmarks, mediciones independientes.   
    for i = 1:size(z, 2)
        landmark_position = [l(z(i).id).x, l(z(i).id).y];
        measurement_range = [z(i).range];

        %% TODO: compute weight
        %p(z |x,l). El desvío estándar de la medición es sigma = 0,2.
         z_real= vecnorm([x(:,1), x(:,2)] - landmark_position, 2, 2);      %Distancia de cada particula al landmark i.
        Vdist = measurement_range - z_real;                                %Rango esperado al landmark i - rango medido/part.
        weight = weight .* normpdf(Vdist, 0, sigma);                       %Multiplico miembro a miembro los pesos(indep).
               
    end

    weight = weight ./ size(z, 2);
end
