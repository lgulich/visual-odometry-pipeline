function robot_pose_vo = tf2RobotPose(T_W_C, theta)
% Estimates the current robot pose from vo
%
%   :param T_W_C: current camera pose
%   :param theta: current tilt angle
%
%   :return robot_pose_vo: vector, the estimated camera robot pose
%
% Estimates the current robot pose from the current visual odometry camera
% transformation matrix

%% correct tf matrix
% compute current tf matrix to ground
con_adj_t = [0 -1 0; 0 0 -1; 1 0 0];
trans_init = [133.1203e-3; -0.2500e-3; 381.9169e-3];
rot_init = [0.9998         0   -0.0193;...
                         0    1.0000         0;...
                         0.0193         0    0.9998];
tans_corr = rotx(-rad2deg(theta))*con_adj_t*trans_init;
            rot_corr = rotx(rad2deg(theta))*rot_init;
tf_curr = [rot_corr, tans_corr; 0 0 0 1];

% correct camera tf matrix
T_W_C_corr = tf_curr\T_W_C;
            
%% assemble pose by projection
robot_pose_vo(1) = T_W_C_corr(1,4);
robot_pose_vo(2) = T_W_C_corr(3,4);
euler_pose = rotm2eul(T_W_C_corr(1:3,1:3));
robot_pose_vo(3) = euler_pose(2);
robot_pose_vo(4) = theta;

end

