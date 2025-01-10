function new_particles = resample(particles, weights)
    % Returns a new set of particles obtained by performing
    % stochastic universal sampling.
    %
    % particles (M x D): set of M particles to sample from. Each row contains a state hypothesis of dimension D.
    % weights (M x 1): weights of the particles. Each row contains a weight.
    new_particles = [];

    M = size(particles, 1);
    
    %% TODO: complete this stub
    
    %Inicilizo el umbral.
    ui= rand/M; %numero aleatorio entre [0-M^-1]
    
    for j= 1:M
        
        idx= find(ui <= cumsum(weights),1);
        new_particles(j,:)= particles(idx , : );
        
        ui= ui + 1/M;

    end
    
end
