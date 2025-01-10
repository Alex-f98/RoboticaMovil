function particles = correction_step(particles, z)

% Weight the particles according to the current map of the particle
% and the landmark observations z.
% z: struct array containing the landmark observations.
% Each observation z(j) has an id z(j).id, a range z(j).range, and a bearing z(j).bearing
% The vector observedLandmarks indicates which landmarks have been observed
% at some point by the robot.

% Number of particles
numParticles = length(particles);

% Number of measurements in this time step
m = size(z, 2);

% TODO: Construct the sensor noise matrix Q_t (2 x 2)
Q_t = [0.1 0; 0 0.1];


% process each particle
for i = 1:numParticles
  robot = particles(i).pose;
  %Posicion cartesiana del robot:
  x_r = robot(1);
  y_r = robot(2);
  theta_r = robot(3);
  % process each measurement
  for j = 1:m
    % Get the id of the landmark corresponding to the j-th observation
    % particles(i).landmarks(l) is the EKF for this landmark
    l = z(j).id; %<--------------------------------------------------j=c_t-

    % The (2x2) EKF of the landmark is given by
    % its mean particles(i).landmarks(l).mu
    % and by its covariance particles(i).landmarks(l).sigma

    % If the landmark is observed for the first time:
    if (particles(i).landmarks(l).observed == false)

      % TODO: inicializa su posición en función de la medición y la pose actual del robot:
      particles(i).landmarks(l).mu = [x_r + z(j).range*cos(z(j).bearing + theta_r);
                                      y_r + z(j).range*sin(z(j).bearing + theta_r)];
                                  
      % obtener el jacobiano con respecto a la posición del landmark
      [h, H] = measurement_model(particles(i), z(j));

      % TODO: inicializar la covarianza para este landmark
      particles(i).landmarks(l).sigma = H\Q_t * inv(H)';

      % Indicar que este lanrdmark ha sido observado
      particles(i).landmarks(l).observed = true;
      
      %w = po ya fue inicializado por fuera.

    else

      % obtener la medición esperada.
      [expectedZ, H] = measurement_model(particles(i), z(j)); %H es 2x2
      % El (2x2) EKF de los landmark esta dado por
      % sus medias: particles(i).landmarks(l).mu
      % y su covarianza: particles(i).landmarks(l).sigma

      % TODO: compute the measurement covariance
      SigmaEKF_li= particles(i).landmarks(l).sigma;
      Q = H * SigmaEKF_li * H' + Q_t;

      % TODO: calculate the Kalman gain
      K = SigmaEKF_li * H' / Q;

      % TODO: compute the error between the z and expectedZ (remember to normalize the angle using the function normalize_angle())
      % expectedZ = [expectedZ_Range; expectedZ_Bearing];POR CADA MEDICIONj
      
      %No se sabe cual angulo normalizar asi que se normalizan las dos
      %antes de computar la diferencia.
      expectedZ(2)= normalize_angle( expectedZ(2) );
      diffZ = [z(j).range; normalize_angle( z(j).bearing )] - expectedZ;
                         

      % TODO: update the mean and covariance of the EKF for this landmark
      particles(i).landmarks(l).mu = particles(i).landmarks(l).mu + K*diffZ;
      particles(i).landmarks(l).sigma = (eye(2) - K*H) * SigmaEKF_li;

      % TODO: compute the likelihood of this observation, multiply with the former weight
      %       to account for observing several features in one time step
  
     particles(i).weight= particles(i).weight *(det(2*pi*Q)^-0.5) * exp(-0.5*(diffZ'/Q)*diffZ);
     %particles(i).weight= particles(i).weight * mvnpdf(diffZ',0,sqrt(Q));
    end

  end % measurement loop
end % particle loop

end
