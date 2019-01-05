function S_out = triangulateNewLandmarks(I_curr, I_prev, S_in, params, T_W_C_in)

% This function triangulates new landmarks as a condition is satisfied.
% The state at the i-th frame is represented as suggested in the mini-pro-
% ject statement: S = (P, X, C, F, T). 
%
% ------------------------------------------------------------------------
%   Inputs:     - I_curr(NxM)       :    current image
%               - I_prev(NxM)       :    previous image
%               - S_in              :    struct containing the variables of 
%                                        the previous state
%               - params            :    parameters struct
%
%   Outputs:    - S_out             :    struct containing the variables of 
%                                        the current state
% ------------------------------------------------------------------------

if ~isempty(S_in.C)
    % Track the candidate keypoints from the previous frame
    pointTracker = vision.PointTracker('MaxBidirectionalError', params.lambda, ...
                                   'NumPyramidLevels', params.num_pyr_levels, ...
                                   'BlockSize', params.bl_size, ...
                                   'MaxIterations', params.max_its);
    initialize(pointTracker, S_in.C.', I_prev);     

    [trackedCandidateKeypoints, isTracked] = step(pointTracker, I_curr);
    trackedCandidateKeypoints = trackedCandidateKeypoints(isTracked,:).';
    
    % Add new landmarks and their associated keypoints to the state, i.e. 
    % update X^i and P^i, if and only if the bearing vectors from the
    % tracked candidate landmarks to the origin of the camera frames where
    % c^i and f^i belongs to form angles greater than a certain threshold
    % alpha (see problem statement 4.3)
    S_in.F = S_in.F(:,isTracked);
    S_in.T = S_in.T(:,isTracked);
    [newValidKeypointsIdx,newLandmarks] = isNewKeypoint( ...
            trackedCandidateKeypoints, S_in.F, T_W_C_in, S_in.T, params ); 
            
    % Update the state eventually removing elements from C^i, F^i and T^i 
    % and adding elements to P^i and X^i.
    S_in.P = [S_in.P, trackedCandidateKeypoints(:,newValidKeypointsIdx)];
    S_in.X = [S_in.X, newLandmarks(:,newValidKeypointsIdx)];
    
    S_in.C = trackedCandidateKeypoints(:,~newValidKeypointsIdx);
    S_in.F = S_in.F(:,~newValidKeypointsIdx);
    S_in.T = S_in.T(:,~newValidKeypointsIdx);
    
    release(pointTracker)
end

% Detect N new Harris corners, where N is arbitrarily defined as the
% difference between a certain number of desired keypoints and the sum of
% the existing keypoints and the candidate ones. 
% 1. First we detect any Harris keypoints
% 2. We choose the k strongest keypoints by metric
% 3. Of the remaining (non-strongest) keypoints we uniformly select N-k
% keypoints
% 4. the N new Harris corners are the union of the strongest and uniformly
% selected keypoints
num_new_corners = max([params.n_keypoints - length(S_in.P) - length(S_in.C); 1]);
corners = detectHarrisFeatures(I_curr, ...
                               'MinQuality', params.feature_quality, ...
                               'ROI', params.ROI, ...
                               'FilterSize', params.filt_size);
strongest_corners = corners.selectStrongest(round(params.strong_to_uniform_kp_ratio*num_new_corners));
[~, is_not_in_strong] = setdiff(corners.Location(:,1)*size(I_curr, 2)+corners.Location(:,2), strongest_corners.Location(:,1)*size(I_curr, 2)+strongest_corners.Location(:,2));
uniform_corners = selectUniform(corners(is_not_in_strong), num_new_corners-size(strongest_corners, 1), size(I_curr));    % TODO

corners = [strongest_corners; uniform_corners];
corners = double(corners.Location.');

% Update the state sets C^i, F^i and T^i such that the new candidate 
% keypoints are not rendundant with existing keypoints in P^i and candidate
% keypoints in C^i
newCandidateKeypoints = ...
    selectNewCandidateKeypoints(corners, S_in.P, S_in.C, params);  

S_in.C = [S_in.C, newCandidateKeypoints];
S_in.F = [S_in.F, newCandidateKeypoints];
num_new_cand_kpts = size(newCandidateKeypoints, 2);
S_in.T = [S_in.T, repmat(T_W_C_in(:), [1,num_new_cand_kpts])];

% Return the updated state
S_out = S_in;

end