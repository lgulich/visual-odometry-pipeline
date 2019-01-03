function [d_robot_pose_W, kalman_state_curr] = estimateDRobotPose(d_robot_pose_vo, d_robot_pose_wo, kalman_state_prev);
% Estimate the current robot pose difference
%
%   :param d_robot_pose_vo: vector, robot pose of the last timestep based
%   on visual odometry
%   :param d_robot_pose_wo: vector, robot pose of the last timestep based
%   on wheel odometry
%   :param kalman_state_prev: struct, the previous kalman state, containing the
%   previous state vector and variance matrix
%
%   :return d_robot_pose_W: vector, the estimated world robot pose difference
%   :return kalman_state_curr: the current kalman state, containing the
%   previous state vector and variance matrix
%
% Fuzes the estimates of the robot pose difference from visual and wheel
% odometry and estimates the scale drift factor

% estimate the current kalman state
% define quantities

% X = kalman_state_prev.X;
% x = X(1);
% z = X(2);
% gamma = X(3);
% beta = X(4);
% 
% P = kalman_state_prev.P;
% 
% A = eye(4);
% L = eye(4);
% 
% H = [

% update kalman state
kalman_state_curr = kalman_state_prev;


% assemble the world robot pose
d_robot_pose_W = (d_robot_pose_vo + d_robot_pose_wo)/2;

end

