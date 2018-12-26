function init_state = initializeState(init_keypoints, init_landmarks)

% Initialization of the state.
%
% ------------------------------------------------------------------------
%   Inputs:     - keypoints_0(2xM)  :    initial keypoints
%               - landmarks_0(3xM)  :    initial landmarks
%
%   Outputs:    - S_0               :    struct containing the variables of 
%                                        the initial state
% ------------------------------------------------------------------------

assert(size(init_keypoints,1) == 2);
assert(size(init_landmarks,1) == 3);

init_state.P = init_keypoints;
init_state.X = init_landmarks;
init_state.C = [];
init_state.F = [];
init_state.T = [];
end