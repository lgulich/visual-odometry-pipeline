function [pose, kp_inliers_idx] = getRelativePose(matched_kp_1,matched_kp_2, params)
% get the relative pose between the images from the matched keypoints
%   :param matched_kp_1
%   :param matched_kp_2
%   :param params struct, general parameter struct

% estimate the fundamental matrix using ransac
[F, kp_inliers_idx, status] = estimateFundamentalMatrix(matched_kp_1, matched_kp_2, 'Method','RANSAC', 'NumTrials',2000, 'DistanceThreshold',1e-4, 'InlierPercentage', 50);


% get the relative camera pose between images using only the keypoint inliers
inlierPoints1 = matched_kp_1(kp_inliers_idx, :);
inlierPoints2 = matched_kp_2(kp_inliers_idx, :);
[R, T, valid_points_fraction] = relativeCameraPose(F, params.cam, inlierPoints1, inlierPoints2);

if valid_points_fraction < 0.5
    '[getRelativePose] WARNING: relative pose might be false, measured low fraction'
end

% combine orientation and translation to single homogenius transformation20
pose = [R, T';
        0 0 0 1];
end

