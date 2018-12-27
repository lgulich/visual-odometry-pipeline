function [newValidKeypointsIdx, newLandmarks] = isNewKeypoint( ...
                    cand_kpts2, cand_kpts_k, T_W_C2, T_W_Ck, params)
    
% This function checks whether the condition for new landmarks to be
% created is satisfied. In turn, these correspond to new keypoints.
%
% ------------------------------------------------------------------------
%   Inputs:     - cand_kpts2(2xL)    :    matched keypoints in the current 
%                                         view
%               - cand_kpts_k(2xL)   :    matched keypoints in the first 
%                                         view each candidate keypoint
%                                         appeared on
%               - T_W_C2(3x4)        :    current camera pose w.r.t. the
%                                         world frame 
%               - T_W_Ck(12xL)       :    first camera pose w.r.t. the
%                                         world frame for each candidate
%                                         keypoint 
%               - params             :    parameters struct
%
%   Outputs:    - newValidLandmarksIdx(1xL)  :   indexes of the new valid 
%                                                landmarks
%               - newLandmarks(3:L)     :     new landmarks
% ------------------------------------------------------------------------

% Check the input dimensions
assert(isequal([3,4],size(T_W_C2)) && size(T_W_Ck,1)==12, ...
                                                'Wrong input dimensions.');
assert(isequal(size(cand_kpts_k),size(cand_kpts2)) && size(cand_kpts_k,1)==2);

% Triangulate the candidate landmarks
[R2,T2] = cameraPoseToExtrinsics(T_W_C2(:,1:3), T_W_C2(:,end));
M2 = cameraMatrix(params.cam, R2, T2);      % projection matrix 

num_kpts = size(cand_kpts2, 2);
newLandmarks = zeros(3,num_kpts);
newValidKeypointsIdx = false(1,num_kpts);
for idx = 1:num_kpts
    T_W_Cidx = reshape(T_W_Ck(:,idx), [3,4]);
    [Ridx,Tidx] = cameraPoseToExtrinsics(T_W_Cidx(:,1:3), T_W_Cidx(:,end));
    Midx = cameraMatrix(params.cam, Ridx, Tidx);
    
    newLandmarks(:,idx) = ...
        triangulate(cand_kpts_k(:,idx).', cand_kpts2(:,idx).', Midx, M2).';
    
    % Check the condition on the angle
    r1 = T_W_Cidx(:,end) - newLandmarks(:,idx);
    r2 = T_W_C2(:,end) - newLandmarks(:,idx);
    angle = atan2d(norm(cross(r1,r2)), dot(r1,r2));         % angle (°)
    if abs(angle) > params.min_angle
        newValidKeypointsIdx(idx) = true;
    end
end

end