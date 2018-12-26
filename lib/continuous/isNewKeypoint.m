function [newLandmarksIdx, newLandmarks] = isNewKeypoint( ...
                    cand_kpts2, cand_kpts1, T_W_C2, T_W_C1, params)
    
% This function checks whether the condition for new landmarks to be
% created is satisfied.
%
% ------------------------------------------------------------------------
%   Inputs:     - cand_kpts2(2xM)    :    matched keypoints in the second 
%                                         view
%               - cand_kpts1(2xM)    :    matched keypoints in the first 
%                                         view
%               - T_W_C2(16x1)        :    second camera pose w.r.t. the
%                                         world frame 
%               - T_W_C1(16x1)        :    first camera pose w.r.t. the
%                                         world frame 
%               - params             :    parameters struct
%
%   Outputs:    - idx                :    indexes of the new landmarks 
% ------------------------------------------------------------------------

% Check the input dimensions
assert(isequal(size(T_W_C1),size(T_W_C2)) && size(T_W_C1,1)==16);
assert(isequal(size(cand_kpts1),size(cand_kpts2)) && size(cand_kpts1,1)==2);

% Triangulate the candidate landmarks
T_W_C1 = reshape(T_W_C1, [4,4]);
T_W_C2 = reshape(T_W_C2, [4,4]);
[R1,T1] = cameraPoseToExtrinsics(T_W_C1(1:3, 1:3), T_W_C1(1:3, 4));
[R2,T2] = cameraPoseToExtrinsics(T_W_C2(1:3, 1:3), T_W_C2(1:3, 4));
M1 = cameraMatrix(params.cam, R1, T1);
M2 = cameraMatrix(params.cam, R2, T2);
newLandmarks = triangulate(cand_kpts1, cand_kpts2, M1, M2).';
assert(size(newLandmarks,1)==3);

% Check the condition
num_kpts = size(cand_kpts1, 2);
newLandmarksIdx = zeros(num_kpts,1);
for idx = 1:num_kpts
    r1 = T_W_C1(1:3, 4) - newLandmarks(:,idx);
    r2 = T_W_C2(1:3, 4) - newLandmarks(:,idx);
    angle = atan2d(norm(cross(r1,r2)), dot(r1,r2));         % angle (°)
    if abs(angle) > params.min_angle
        newLandmarksIdx(idx) = 1;
    end
end

end