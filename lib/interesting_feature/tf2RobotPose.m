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




robot_pose_vo(1) = 0;
robot_pose_vo(2) = 0;
robot_pose_vo(3) = 0;
robot_pose_vo(4) = theta;

end

