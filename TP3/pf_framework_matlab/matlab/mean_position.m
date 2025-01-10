function mean_pos = mean_position(particles, weights)
    % Returns a single estimate of filter state based on the particle cloud.
    %
    % particles (M x 3): set of M particles to sample from. Each row contains a state hypothesis of dimension 3 (x, y, theta).
    % weights (M x 1): weights of the particles. Each row contains a weight.

    % initialize
    mean_pos = zeros(1,3);

    %% TODO: compute mean_pos 
%     aux = zeros(size(particles));
%     for i = 1:size(particles,1) 
%         aux(i,:) = particles(i,:)*weights(i);
%     end
%     mean_pos = sum(aux)/sum(weights);

    %mean_pos= [particles(:,1)'*weights, particles(:,2)'*weights, particles(:,3)'*weights] / sum(weights);
    mean_pos= particles(:,:)'*weights / sum(weights)';
    %mean_pos= mean(particles);
    %preguntar como juega el peso aca? suma ponderada?-> weights =[0.5 0.5]
end
