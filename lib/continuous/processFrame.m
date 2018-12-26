function [S_curr, T_W_C_curr] = processFrame(I_curr, I_prev, S_prev, params)

% Implementation of the VO continuous operation. Each frame is processed
% according to the following three steps: 1) keypoints in the current frame
% are associated to previously triangulated landmarks, 2) the current
% camera pose is subsequently estimated and 3) new landmarks are
% triangulated.
% The state at the i-th frame is represented as suggested in the mini-pro-
% ject statement: S = (P, X, C, F, T). 
%
% ------------------------------------------------------------------------
%   Inputs:     - I_curr(NxM)       :    current image
%               - I_prev(NxM)       :    previous image
%               - S_prev            :    struct containing the variables of 
%                                        the previous state
%               - params            :    parameters struct
%
%   Outputs:    - S_curr            :    struct containing the variables of 
%                                        the current state
%               - T_WC_curr(16x1)   :    current camera pose w.r.t. the
%                                        world frame
% ------------------------------------------------------------------------

% Sanity check
state_fields = {'P', 'X', 'C', 'F', 'T'};
params_fields = {'lambda', 'min_angle', 'new_cand_kpt_threshold', 'cam'};
assert(all(isfield(S_prev, state_fields)), ...
                        'Some fields are missing in the state struct.');
assert(all(isfield(params, params_fields)), ...
                        'Some fields are missing in the parameters struct.');
                    
% Process the frame
[S_curr.P, T_W_C_curr] = ...
    estimateCurrentPose(I_curr, I_prev, S_prev.P, S_prev.X, params);               
S_curr = triangulateNewLandmarks(I_curr, I_prev, S_curr, params, T_W_C_curr);

end