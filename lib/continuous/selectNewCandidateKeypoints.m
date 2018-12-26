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
new_cand_kpts_idx = ones(size(corners, 2), 2);
for corner_idx = 1:size(corners, 2)
    corner = corners(:,corner_idx);
    for kpt_idx = 1:size(kpts, 2)
        for cand_kpt_idx = 1:size(cand_kpts, 2)
            if norm(corner - kpts(kpt_idx)) < params.new_cand_kpt_threshold && ...
               norm(corner - cand_kpts(cand_kpt_idx)) < params.new_cand_kpt_threshold
                new_cand_kpts_idx(corner_idx) = 0;
                break
            end
        end
        if new_cand_kpts_idx(corner_idx) == 0
            break
        end
    end
end
newCandidateKeypoints = corners(:,new_cand_kpts_idx);
end