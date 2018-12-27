function [curr_kpts, curr_landmarks, T_W_C_curr] = ...
    estimateCurrentPose(I_curr, I_prev, prev_kpts, prev_landmarks, params)

% Estimate the pose of the camera with respect to the world frame.
%
% ------------------------------------------------------------------------
%   Inputs:     - I_curr(NxM)       :    current image
%               - I_prev(NxM)       :    previous image
%               - prev_kpts(2xK)    :    keypoints corresponding to the 
%                                        landmarks in the previous frame
%               - landmarks(3xK)    :    landmarks 
%               - params            :    parameters struct
%
%   Outputs:    - curr_kpts(2xK)    :    keypoints corresponding to the 
%                                        landmarks in the current frame
%               - T_W_C_curr(3x4)   :    current camera pose w.r.t. the
%                                        world frame
% ------------------------------------------------------------------------

% Sanity check
assert(size(prev_kpts, 2) == size(prev_landmarks, 2));

% Track the keypoints from the previous frame
pointTracker = vision.PointTracker('MaxBidirectionalError', params.lambda);
initialize(pointTracker, prev_kpts.', I_prev);     

[trackedKeypoints, isTracked] = step(pointTracker, I_curr);
curr_kpts = trackedKeypoints(isTracked,:).';        % (2xS)
curr_landmarks = prev_landmarks(:,isTracked);            % (3xS)
assert(size(curr_kpts, 2) == size(curr_landmarks, 2));

% Estimate the camera pose in the world coordinate system
[R, T, ~, status] = ...
    estimateWorldCameraPose(curr_kpts.', curr_landmarks.', params.cam, ...
        'MaxNumTrials', 8000, 'Confidence', 84, 'MaxReprojectionError', 2);
assert(status == 0, sprintf('Error in P3P: status = %d.', status));

% Combine orientation and translation into a single transformation matrix
T_W_C_curr = [R, T.'];

end