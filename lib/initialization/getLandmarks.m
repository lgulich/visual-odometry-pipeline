function [landmarks] = getLandmarks(matched_kp_1, matched_kp_2, pose, params)
% Triangulate the 2d keypoints from two images to obtain a 3d pointcloud
%
%   :param matched_kp_1: class cornerPoint, the matching keypoints from the
%   first image
%   :param matched_kp_2: class cornerPoint, the matching keypoints from the
%   second image
%   :param pose: 4x4 matrix, the homogenous transformation of the camera
%   position between the first and second image
%   :param params: struct, the parameter struct
%
%   :return landmarks: 3xN matrix, 3d-position of the keypoints in the
%   worldframe

assert((size(pose, 1))==4)
assert((size(pose, 2))==4)

% Compute the camera matrices for each position of the camera
% The first camera is at the origin looking along the Z-axis. Thus, its
% rotation matrix is identity, and its translation vector is 0.
cam_matrix_1 = cameraMatrix(params.cam, eye(3), [0 0 0]);

% Compute extrinsics of the second camera
[R, t] = cameraPoseToExtrinsics(pose(1:3, 1:3), pose(1:3, 4));
cam_matrix_2 = cameraMatrix(params.cam, R, t);

% Compute the 3-D points
landmarks = triangulate(matched_kp_1, matched_kp_2, cam_matrix_1, cam_matrix_2);
landmarks = landmarks';

end

