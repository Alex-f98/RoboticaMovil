% Make librobotics available
addpath('librobotics');
addpath('ffmpeg-r8');

% Read world data, i.e. landmarks
fprintf('Reading world data ...');
landmarks = read_world('../data/world.dat');
fprintf(' done\n');
% Read sensor readings, i.e. odometry and range-bearing sensor
fprintf('Reading sensor data ...');
data = read_data('../data/sensor_data.dat');
fprintf(' done\n');

% Initialize particles
particles = initialize_particles(500);

function [new_pose]= particle_filter(dd, v_cmd, w_cmd, particles, weight)

%for t = 1:size(data.timestep, 2)

    new_particles = pf.sample_motion_model(dd, v_cmd, w_cmd, pose, sampleTime);
    weights = measurement_model(data.timestep(t).sensor, new_particles, landmarks);
    
    normalizer = sum(weights);
    weights = weights ./ normalizer;

    plot_state(new_particles, weights, landmarks, t);
    
    particles = resample(new_particles, weights);
%end

%fprintf('Final pose: ')
new_pose = mean_position(particles, weights))

end
