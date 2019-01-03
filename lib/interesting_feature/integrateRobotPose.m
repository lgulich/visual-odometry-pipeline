function robot_pose_curr = integrateRobotPose(robot_pose_last, d_robot_pose, theta_curr)
% Assemble the current pose
%
%   :param robot_pose_last: vector, robot pose of the last timestep
%   :param d_robot_pose: vector, difference between robot poses of the
%   current and last timesteps
%   :param theta_curr: the current tilt angle (by convention)
%
% create a robot pose based on the last pose and the difference, following
% the convention

% add up difference
robot_pose_curr(1) = robot_pose_last(1) + d_robot_pose(1);
robot_pose_curr(2) = robot_pose_last(2) + d_robot_pose(2);
robot_pose_curr(3) = robot_pose_last(3) + d_robot_pose(3);

% set by convention
robot_pose_curr(4) = theta_curr;

end

