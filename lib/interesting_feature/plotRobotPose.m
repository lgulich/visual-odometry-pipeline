function patch_out = plotRobotPose(r_p,color,image,map_size, patch_in)
% Plot the the current pose of the robot
%
%   :param rp: vector, robot pose
%   :param color: string, the desired color of the robot pose
%   :param image: image, corresponding image
%   :param map_size: double, 0: for no replot, pos: for rectangle scale, 
%   neg: for robot follower with -rectangle scale
%   :param patch_in: bool, a flag if it's the first time plotting
%
%   :return patch_out: bool, a flag if it's the first time plotting
%
% plot a 3d world with a robot model inside and the corresponding image
% below

% set figure (figure 1 is the vo plot)
figure(2);

%% plot corresponding image
subplot(2,2,[3,4]);
imshow(image, 'InitialMagnification', 'fit');
title('Image as reference');
drawnow;

%% plot trajectory
subplot(2,2,2);
title('Global trajectory');

% set up plot
if map_size == 0
else
    grid on
    axis equal
    axis manual
    hold on
    xlabel('x');
    ylabel('y');
    zlabel('z');
    view(0,-50);
    xlim([-1,25]);
    ylim([-1.5,0.1]);
    zlim([-1,25]);
    view(0,0);
end

% plot floor if it's the first time
if patch_in == true
    patch([-50 50 50 -50], [0 0 0 0], [-50 -50 50 50], 'black','FaceAlpha',0.5);
end
patch_out = false;

% calculate and plot ground point
gp = [r_p(1); 0; r_p(2)];
plot3(gp(1), gp(2), gp(3), [color, 'x'], 'MarkerSize', 8);

%% plot robot pose
subplot(2,2,1);
title('Robot poses in the current vicinity');

% set up plot
if map_size == 0
else
    grid on
    axis equal
    axis manual
    hold on
    xlabel('x');
    ylabel('y');
    zlabel('z');
    view(0,-50);
    if map_size <  0
        xlim([map_size+r_p(1),-map_size+r_p(1)]);
        ylim([-1.5,0.1]);
        zlim([map_size+r_p(2),-map_size+r_p(2)]);
    else
        xlim([-map_size,map_size]);
        ylim([-1.5,0.1]);
        zlim([-map_size,map_size]);
    end
end

% plot floor if it's the first time
if patch_in == true
    patch([-50 50 50 -50], [0 0 0 0], [-50 -50 50 50], 'black','FaceAlpha',0.5);
end
patch_out = false;

% set geometric params
w_r = 0.075; % wheel radius
w_d = 0.1625; % wheel distance to center axis

% calculate and plot ground point
gp = [r_p(1); 0; r_p(2)];
plot3(gp(1), gp(2), gp(3), ['w', 'x'], 'MarkerSize', 2);

% calculate midpoint between the wheels
mp = gp + [0; -w_r; 0];

% calculate midpoints of the wheels
lw_mp = mp + w_d*[cos(r_p(3)); 0; sin(r_p(3))];
rw_mp = mp - w_d*[cos(r_p(3)); 0; sin(r_p(3))];

% calculate and plot wheels
theta = [(0:0.1:2*pi), 0];
rw_t = rw_mp + w_r*([0;-1; 0]*cos(theta)+[-sin(r_p(3)); 0; cos(r_p(3))]*sin(theta)); % tire
lw_t = lw_mp + w_r*([0;-1; 0]*cos(theta)+[-sin(r_p(3)); 0; cos(r_p(3))]*sin(theta)); % tire
plot3(rw_t(1,:).', rw_t(2,:).', rw_t(3,:).',color, 'LineWidth', 2);
plot3(lw_t(1,:).', lw_t(2,:).', lw_t(3,:).',color, 'LineWidth', 2);

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
plotCamera('Location',t_cam,'Orientation',R_cam,'Size',0.05,'Color',color);
% plot3(t_cam(1), t_cam(2), t_cam(3), [color, 'o']);
plot3([lw_mp(1); t_cam(1)], [lw_mp(2); t_cam(2)],...
    [lw_mp(3); t_cam(3)], color, 'LineWidth', 2);
plot3([rw_mp(1); t_cam(1)], [rw_mp(2); t_cam(2)],...
    [rw_mp(3); t_cam(3)], color, 'LineWidth', 2);

% show everything
drawnow;

end

