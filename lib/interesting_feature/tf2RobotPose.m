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

robot_pose_vo(1) = T_W_C(1,4);
robot_pose_vo(2) = T_W_C(3,4);
euler_pose = rotm2eul(T_W_C(1:3,1:3));
robot_pose_vo(3) = euler_pose(2);
robot_pose_vo(4) = theta;

end

