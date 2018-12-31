function [pose, valid_kp, valid_marks] = initialize(img_1,img_2, params)
% Initialize the vo pipeline by calculating an initial estimate of camera
% pose and landmarks
%
%   :param img_1: matrix NxM, the first initial image
%   :param img_2: matrix NxM, the second initial image
%
%   :return p_0: matrix 4x4, the initial pose of the camera
%   :matched_kp_2: matrix 2xN, the keypoints corresponding to the initial
%                  landmarks
%   :return marks: matrix 3xN, 3d-positions in world frame of initial landmarks

% get keypoints of images using harris detector
kp_1 = getKeypoints(img_1, params);
kp_2 = getKeypoints(img_2, params);

% get descriptors of image and update keypoints to only include valid ones
[des_1, valid_kp_1] = getDescriptors(img_1, kp_1, params);
[des_2, valid_kp_2] = getDescriptors(img_2, kp_2, params);

% match keypoints between images
[matched_kp_1, matched_kp_2] = getMatchedKeypoints(des_1, des_2, ...
                            valid_kp_1, valid_kp_2, img_1, img_2, params);

% estimate relative pose between images
[pose, kp_inliers_idx] = getRelativePose(matched_kp_1, matched_kp_2, params);

% triangulate pointclouds
marks = getLandmarks(matched_kp_1, matched_kp_2, pose, params);
valid_marks = marks(:, marks(3,:)>=0);
valid_kp = matched_kp_2(marks(3,:)>=0);

% if the vizualisation is enabled plot the features and pointcloud
if params.viz_enabled
    % plot the matched features before and after RANSAC
    h = figure;
    subplot(3,2,1);
    showMatchedFeatures(img_1,img_2, matched_kp_1, matched_kp_2);
    title('Point matches before outliers were removed');
    
    subplot(3,2,2);
    showMatchedFeatures(img_1, img_2, matched_kp_1(kp_inliers_idx), matched_kp_2(kp_inliers_idx));
    title('Point matches after outliers were removed');
    %waitfor(h) % wait until plot is closed

    % plot point cloud and cameras
    subplot(3,2,3:6)
    hold on
    grid on
    [R, t] = cameraPoseToExtrinsics(pose(1:3, 1:3), pose(1:3, 4));
    cam_1 = plotCamera('Location',[0 0 0], 'Orientation',eye(3), 'Label','Camera1', 'AxesVisible',true, 'Size',2, 'Color',[0,0,0]);
    cam_2 = plotCamera('Location',t ,'Orientation',R , 'Label','Camera2', 'Size',2);
    scatter3(marks(1,:), marks(2,:), marks(3,:), 'filled' )
    %xlim([-5 5]); ylim([-3 3]); zlim([-1 60]);
    xlabel('x'); ylabel('y'); zlabel('z')
    view(0, 0); % view directly from top
    title('3D Scene');

end

end

