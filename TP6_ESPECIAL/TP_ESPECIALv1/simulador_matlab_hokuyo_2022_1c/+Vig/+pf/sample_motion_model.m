function new_pose = sample_motion_model(dd, v_cmd, w_cmd, pose, sampleTime)
    % dd: diferential drive object
    % v_cmd, w_cmd: odometry reading [linear_veloc angular_veloc]
    % pose: set of old particles (Nx3)
    % timestep: timestep of simulation

	new_pose = zeros(size(pose));
	% Mover el robot segun los comandos generados
    [wL,wR] = inverseKinematics(dd,v_cmd,w_cmd);
    % Velocidad resultante
    [v,w] = forwardKinematics(dd,wL,wR);
    velB = [v;0;w]; % velocidades en la terna del robot [vx;vy;w]

    % receives 3xN array
    vel = base.bodyToWorld(velB, pose'); % Conversion de la terna del robot a la global
    % Realizar un paso de integracion
    new_pose = (pose' + vel*sampleTime)';
    
end
