function [pose, marks] = initialize(img_1,img_2, params)
%   Initialize the vo pipeline by calculating an initial estimate of camera
%   pose and landmarks
%   :param img_1 matrix NxM, the first initial image
%   :param img_2 matrix NxM, the second initial image
%   :return p_0 vector 3x1, the initial position of the camera
%   :return marks matrix 3xN, positions in world frame of initial landmarks

% get keypoints of images using harris detector
kp_1 = getKeypoints(img_1, 'harris', params);
kp_2 = getKeypoints(img_2, 'harris', params);

% debug show keypoints
%imshow(img_1); hold on; plot(kp_1)

% get descripors of image and update keypoints to only include valid ones
[des_1, valid_kp_1] = getDescriptors(img_1, kp_1, params);
[des_2, valid_kp_2] = getDescriptors(img_2, kp_2, params);

% match keypoints between images
[matched_kp_1, matched_kp_2] = getMatchedKeypoints(des_1, des_2, valid_kp_1, valid_kp_2);

% debug show matches
%figure; showMatchedFeatures(img_1,img_2, matched_kp_1, matched_kp_2);

% estimate relative pose between images
[pose, kp_inliers_idx] = getRelativePose(matched_kp_1, matched_kp_2, params);

% debug outlier filtered matches
figure; showMatchedFeatures(img_1, img_2, matched_kp_1(kp_inliers_idx), matched_kp_2(kp_inliers_idx));

% triangulate pointclouds
marks = getLandmarks(matched_kp_1, matched_kp_2, pose, params);

% debug show 3d structure
% hold on;
% cam_1 = plotCamera('Location',[0 0 0],'Orientation', eye(3), 'Label', '1', 'AxesVisible', true);
% cam_2 = plotCamera('Location', pose(1:3, 4) ,'Orientation', pose(1:3, 1:3), 'Label', '2');
% scatter3(marks(1,:), marks(2,:), marks(3,:), 'filled' )
% xlim([0 10]); ylim([0 10]); zlim([0 10]);
% xlabel('x'); ylabel('x'); zlabel('z')

end

