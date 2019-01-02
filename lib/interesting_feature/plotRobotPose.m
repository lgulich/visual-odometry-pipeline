function patch_out = plotRobotPose(r_p,color,image,patch_in)
% Plot the the current pose of the robot
%
%   :param rp: vector, robot pose
%   :param color: string, the desired color of the robot pose
%   :param image: image, corresponding image
%   :param patch_in: bool, a flag if it's the first time plotting
%
%   :return patch_out: bool, a flag if it's the first time plotting
%
% plot a 3d world with a robot model inside and the corresponding image
% below


% set figure (figure 1 is the vo plot)
figure(2);

%% plot corresponding image
subplot(2,1,2);
imshow(image, 'InitialMagnification', 'fit');
drawnow;

%% plot robot pose
subplot(2,1,1)

% set up plot
map_size = 0; % 0: for whole map, number: for rectangle scale
grid on
axis equal
axis manual
hold on
if map_size ==  0
    xlim([-1,20]);
    ylim([-0.5,1.5]);
    zlim([-1,20]);
else
    xlim([-map_size,map_size]);
    ylim([-0.5,1.5]);
    zlim([-map_size,map_size]);
end
xlabel('x');
ylabel('y');
zlabel('z');
view(0,-50);

% plot floor if it's the first time
if patch_in == true
    patch([-50 50 50 -50], [0 0 0 0], [-50 -50 50 50], [0.1 0.1 0.1 0.1]);
end
patch_out = false;

% set geometric params
w_r = 0.075; % wheel radius
w_d = 0.1625; % wheel distance to center axis

% calculate and plot ground point
gp = [r_p(1); 0; r_p(2)];
plot3(gp(1), gp(2), gp(3), [color, 'x']);

% calculate and plot midpoint between the wheels
mp = gp + [0; -w_r; 0];
plot3(mp(1), mp(2), mp(3), [color, 'o']);

% calculate and plot midpoints of the wheels
lw_mp = mp + w_d*[cos(r_p(3)); 0; sin(r_p(3))];
rw_mp = mp - w_d*[cos(r_p(3)); 0; sin(r_p(3))];
plot3([lw_mp(1); rw_mp(1)], [lw_mp(2); rw_mp(2)],...
    [lw_mp(3); rw_mp(3)], color);

% calculate and plot wheels
theta = [(0:0.1:2*pi), 0];
rw_t = rw_mp + w_r*([0;-1; 0]*cos(theta)+[-sin(r_p(3)); 0; cos(r_p(3))]*sin(theta)); % tire
lw_t = lw_mp + w_r*([0;-1; 0]*cos(theta)+[-sin(r_p(3)); 0; cos(r_p(3))]*sin(theta)); % tire
plot3(rw_t(1,:).', rw_t(2,:).', rw_t(3,:).',color);
plot3(lw_t(1,:).', lw_t(2,:).', lw_t(3,:).',color);

% specify default camera translation and rotation
cam_t = [133.1203e-3; -0.2500e-3; 381.9169e-3];
cam_R = [0.9998         0   -0.0193;...
    0    1.0000         0;...
    0.0193         0    0.9998];

% calculate current camera translation and rotation
con_adj_t = [0 -1 0; 0 0 -1; 1 0 0];
t_cam = gp + roty(-rad2deg(r_p(3)))*rotx(-rad2deg(r_p(4)))*con_adj_t*cam_t;
R_cam = rotx(rad2deg(r_p(4)))*roty(rad2deg(r_p(3)))*cam_R;

% plot camera
cam = plotCamera('Location',t_cam,'Orientation',R_cam,'Size',0.05,'Color',color);
plot3(t_cam(1), t_cam(2), t_cam(3), [color, 'o']);

% show everything
drawnow;

end

