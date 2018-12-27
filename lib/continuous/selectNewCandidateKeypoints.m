function newCandidateKeypoints = selectNewCandidateKeypoints( ...
                                        corners, kpts, cand_kpts, params)

% This function checks and selects new candidate keypoints so that these
% are not redundant with the existing keypoints and candidate ones.
%
% ------------------------------------------------------------------------
%   Inputs:     - corners(2xM)              :    new corners to be evaluated
%               - kpts(2xN)                 :    existing keypoints
%               - cand_kpts(2xR)            :    existing candidate keypoints
%
%   Outputs:    - newCandidateKeypoints     :    new candidate keypoints
% ------------------------------------------------------------------------

% Corners are new candidate keypoints if and only if their distances from
% each existing keypoint and/or candidate keypoint is greater than a
% certain threshold

ex_keypoints = [kpts, cand_kpts];
new_cand_kpts_idx = ones(1, size(corners, 2));
    
for corner_idx = 1:size(corners, 2)
    corner = corners(:,corner_idx);
    sq_distances = sum((ex_keypoints - corner).^2);
    new_cand_kpts_idx(corner_idx) = ...
                    all(sq_distances > params.new_cand_kpt_threshold^2);
end
newCandidateKeypoints = corners(:,logical(new_cand_kpts_idx));

end