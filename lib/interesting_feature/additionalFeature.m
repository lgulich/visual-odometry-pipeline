% Additional feature
%
% uses workspace from main file
%
% Reads & transforms data, calls further functions as e.g. the extended
% kalman filter or the additional plotting scripts

if i > range(1)
    
    % update world iterators
    theta_curr = double(double(ascento_est_states{i*20-2}.EstThetaMean));
    
    % update wo state
    x_curr = double((-ascento_est_states{i*20-2}.EstRobotYPos) - x_initial);
    z_curr = double(ascento_est_states{i*20-2}.EstRobotXPos - z_initial);
    gamma_curr = double(ascento_est_states{i*20-2}.EstRobotGammaOrient - gamma_initial);
    
    robot_pose_wo_curr(1) = x_curr*cos(gamma_initial)+z_curr*sin(gamma_initial);
    robot_pose_wo_curr(2) = z_curr*cos(gamma_initial)-x_curr*sin(gamma_initial);
    robot_pose_wo_curr(3) = gamma_curr;
    robot_pose_wo_curr(4) = theta_curr;
    
    % update vo state
    T_W_C_curr_rec = T_W_C_init_inv * [T_W_C_curr; 0 0 0 1];
    robot_pose_vo_curr = tf2RobotPose(T_W_C_curr_rec, theta_curr);
    
    % get translation vector from wo
    d_robot_pose_wo = robot_pose_wo_curr - robot_pose_wo_last;
    
    % get translation vector from vo
    d_robot_pose_vo = robot_pose_vo_curr - robot_pose_vo_last;
    
    % simulate vo if desired
    if simulate_vo == true
        d_robot_pose_vo(1) = d_robot_pose_wo(1)*(1-i/1000); % for scale drift
        d_robot_pose_vo(2) = d_robot_pose_wo(2)*(1-i/1000); % for scale drift
        d_robot_pose_vo(3) = d_robot_pose_wo(3);
        d_robot_pose_vo(4) = d_robot_pose_wo(4);
    end
    
    % combine translation vectors
    [d_robot_pose_W, kalman_state] = estimateDRobotPose(d_robot_pose_vo, d_robot_pose_wo, theta_curr, kalman_state, i);
    
    % rectify vo state
    d_robot_pose_vo_rec = d_robot_pose_vo./[kalman_state.X(4); kalman_state.X(5); 1; 1].';
    
    
    % update VO state
    robot_pose_vo_curr_rec = integrateRobotPose(robot_pose_vo_last_rec, d_robot_pose_vo_rec, theta_curr);
    
    % update W state
    robot_pose_W_curr = integrateRobotPose(robot_pose_W_last, d_robot_pose_W, theta_curr);
    
    % plot vo, wo and W robot poses
    every = false;
    if(~mod(i,plotting_speed+range(2)) || i == range(2) || every)
        patch_bool = plotRobotPose(robot_pose_W_curr, 'r', image, -1.8, patch_bool);
        patch_bool = plotRobotPose(robot_pose_wo_curr, 'm', image, 0, patch_bool);
        patch_bool = plotRobotPose(robot_pose_vo_curr, 'c', image, 0, patch_bool);
        patch_bool = plotRobotPose(robot_pose_vo_curr_rec, 'b', image, 0, patch_bool);
        plotCamera('Location',T_W_C_curr_rec(1:3,4),'Orientation',T_W_C_curr_rec(1:3,1:3),'Size',0.05,'Color','k');
    end
    
    % update vo iterators
    robot_pose_vo_last = robot_pose_vo_curr;
    robot_pose_vo_last_rec = robot_pose_vo_curr_rec;
    
    % update wo iterators
    robot_pose_wo_last = robot_pose_wo_curr;
    
    % update W iterators
    robot_pose_W_last = robot_pose_W_curr;
    
elseif i == range(1)
    
    % initialize global quantities
    patch_bool = true;
    kalman_state.X = [0.0; 0.0; 0.0; 3; 5];
    kalman_state.P = diag([0.0; 0.0; 0.0; 0.5; 1].^2);
    
    % initialize W iterators
    theta_last = double(ascento_est_states{i*20-2}.EstThetaMean);
    
    robot_pose_W_last(1)= 0.0;
    robot_pose_W_last(2)= 0.0;
    robot_pose_W_last(3)= 0.0;
    robot_pose_W_last(4) = theta_last;
    
    % initialize wo iterators
    robot_pose_wo_last(1)= 0.0;
    robot_pose_wo_last(2)= 0.0;
    robot_pose_wo_last(3)= 0.0;
    robot_pose_wo_last(4) = theta_last;
    
    x_initial = double(-ascento_est_states{range(1)*20-2}.EstRobotYPos);
    z_initial = double(ascento_est_states{range(1)*20-2}.EstRobotXPos);
    gamma_initial = double(ascento_est_states{range(1)*20-2}.EstRobotGammaOrient);
    
    % initialialize vo iterators
    con_adj_t = [0 -1 0; 0 0 -1; 1 0 0];
    trans_init = [133.1203e-3; -0.2500e-3; 381.9169e-3];
    rot_init = [0.9998         0   -0.0193;...
        0    1.0000         0;...
        0.0193         0    0.9998];
    tans_corr = rotx(-rad2deg(theta_last))*con_adj_t*trans_init;
    rot_corr = rotx(rad2deg(theta_last))*rot_init;
    tf_init = [rot_corr, tans_corr; 0 0 0 1];
    T_W_C_init_inv = tf_init/[T_W_C_curr; 0 0 0 1];
    
    robot_pose_vo_last(1)= 0.0;
    robot_pose_vo_last(2)= 0.0;
    robot_pose_vo_last(3)= 0.0;
    robot_pose_vo_last(4) = theta_last;
    
    robot_pose_vo_last_rec = robot_pose_vo_last;
    
end