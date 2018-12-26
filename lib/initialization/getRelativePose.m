function [pose, kp_inliers_idx] = getRelativePose(matched_kp_1,matched_kp_2, params)
% Get the relative pose between the images from the matched keypoints
%
%   :param matched_kp_1: class cornerPoint, the matching keypoints from the
%   first image
%   :param matched_kp_2: class cornerPoint, the matching keypoints from the
%   second image
%   :param params struct: general parameter struct
%
%   :return pose: 4x4 matrix: the relative camera pose between the frames
%   expressed as a homogeneous transformation
%   :return kp_inliers_idx: vector 1xN, bool for every keypoint wheter it
%   is a inlier or not


% estimate the fundamental matrix using RANSAC
[F, kp_inliers_idx, status] = estimateFundamentalMatrix( ...
                                matched_kp_1, matched_kp_2, ...
                                'Method','RANSAC', 'NumTrials', 2000, ...
                                'DistanceThreshold', 1e-4);
assert(status==0);

% get the relative camera pose between images using only the keypoint inliers
inlierPoints1 = matched_kp_1(kp_inliers_idx, :);
inlierPoints2 = matched_kp_2(kp_inliers_idx, :);

[R, T, valid_points_fraction] = relativeCameraPose(F, params.cam, inlierPoints1, inlierPoints2);
 
% check that fraction is big enough
if valid_points_fraction < 0.5
    warning('[getRelativePose] WARNING: relative pose might be false, measured low fraction if %f', valid_points_fraction);
end

% combine orientation and translation to single homogeneus transformation
pose = [R, T';
        0 0 0 1];
end

