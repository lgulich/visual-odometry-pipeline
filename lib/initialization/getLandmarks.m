function [landmarks] = getLandmarks(matched_kp_1, matched_kp_2, pose, params)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
assert((size(pose, 1))==4)
assert((size(pose, 2))==4)

% calculate projection matrices M1, M2
M1 = params.cam.IntrinsicMatrix * pose(1:3, :);
M2 = params.cam.IntrinsicMatrix * pose(1:3, :);

% get keypoint positions
p1 = matched_kp_1(:).Location;
p2 = matched_kp_2(:).Location;

% add row of 1's to make coordinates homogenous
p1 = [p1'; ones(1, size(p1, 1))];
p2 = [p2'; ones(1, size(p2, 1))];

% triangulate 3d points from matched keypoints
landmarks = linearTriangulation(p1,p2,M1,M2);

% cut off last entry of each homogenous coordinate to leave x, y, z
landmarks = landmarks(1:3, :);

end

