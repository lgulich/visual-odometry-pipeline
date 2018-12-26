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

% Track the candidate keypoints from the previous frame
pointTracker = vision.PointTracker('MaxBidirectionalError', params.lambda);
initialize(pointTracker, S_in.C, I_prev);     

[trackedCandidateKeypoints, isTracked] = step(pointTracker, I_curr);
trackedCandidateKeypoints = trackedCandidateKeypoints(:,isTracked);

% Update the state
[newLandmarksIdx,newLandmarks] = isNewKeypoint( trackedCandidateKeypoints, ...
            S_in.F(isTracked), T_W_C_in, S_in.T(isTracked), params );  
S_out.P = [S_in.P, trackedCandidateKeypoints(:,newLandmarksIdx)];
S_out.X = [S_in.X, newLandmarks(:,newLandmarksIdx)];
S_out.C = trackedCandidateKeypoints(:,~newLandmarksIdx);
S_out.F = S_in.F(:,~newLandmarksIdx);
S_out.T = S_in.T(:,~newLandmarksIdx);

% Detect new Harris corners not rendundant with existing keypoints and
% candidate landmarks
corners = detectHarrisFeatures(I_curr).selectStrongest(params.n_keypoints);
corners = corners.location;

newCandidateKeypoints = ...
    selectNewCandidateKeypoints(corners, S_out.P, S_out.C, params);  
num_new_cand_kpts = size(newCandidateKeypoints, 2);

S_out.C = [S_out.C, newCandidateKeypoints];
S_out.F = [S_out.F, newCandidateKeypoints];
S_out.T = [S_out.T, repmat(T_W_C_in, [1,num_new_cand_kpts])];

release(pointTracker);

end