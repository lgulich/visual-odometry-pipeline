function [d_robot_pose_W, kalman_state_curr] = estimateDRobotPose(d_robot_pose_vo, d_robot_pose_wo, theta_curr, kalman_state_prev, i);
% Estimate the current robot pose difference
%
%   :param d_robot_pose_vo: vector, robot pose of the last timestep based
%   on visual odometry
%   :param d_robot_pose_wo: vector, robot pose of the last timestep based
%   on wheel odometry
%   :param theta_curr: the current tilt angle
%   :param kalman_state_prev: struct, the previous kalman state, containing the
%   previous state vector and variance matrix
%
%   :return d_robot_pose_W: vector, the estimated world robot pose difference
%   :return kalman_state_curr: the current kalman state, containing the
%   previous state vector and variance matrix
%
% Fuzes the estimates of the robot pose difference from visual and wheel
% odometry and estimates the scale drift factor

%% define quantities
X = kalman_state_prev.X;
% x = X(1); for reference
% z = X(2); for reference
% gamma = X(3); for reference
% beta = X(4); for reference
% delta = X(5); for reference

Z = [d_robot_pose_vo(1:3).'; d_robot_pose_wo(1:3).'];

P = kalman_state_prev.P;
Q = diag([0.075; 0.075; 0.1; 0.2; 0.8].^2);
R = diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1].^2);

%% prior update
% define linearized matrices
A = eye(5);
L = eye(5);

% compute updated
X_p = A*X;
P_p = A*P*A.' + L*Q*L.';

x_p = X_p(1);
z_p = X_p(2);
gamma_p = X_p(3);
beta_p = X_p(4);
delta_p = X_p(5);

%% measurement update
% define linearized matrices
H = [beta_p 0 0 x_p 0; ...
    0 delta_p 0 0 z_p; ...
    0 0 1 0 0; ...
    1 0 0 0 0; ...
    0 1 0 0 0; ...
    0 0 1 0 0];
M = diag([beta_p delta_p 1 1 1 1]);

% compute update
K = P_p*H.'/(H*P_p*H.' + M*R*M.');
h = [beta_p*x_p; delta_p*z_p; gamma_p; x_p; z_p; gamma_p];

X_m = X_p + K*(Z-h);
P_m = (eye(5)-K*H)*P_p;

%% write new kalman state
kalman_state_curr.X = X_m;
kalman_state_curr.P = P_m;

% plot scale factors
% figure(3)
% hold on
% plot(i, X_m(4), 'bo');
% plot(i, X_m(5), 'ro');

%% assemble the world robot pose
d_robot_pose_W = [kalman_state_curr.X(1); kalman_state_curr.X(2); ...
    kalman_state_curr.X(3); theta_curr];

end

