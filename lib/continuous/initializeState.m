function init_state = initializeState(init_keypoints, init_landmarks)

% Initialization of the state.
%
% ------------------------------------------------------------------------
%   Inputs:     - init_keypoints    :    initial keypoints (cornerPoints
%                                        object)
%               - init_landmarks(3xM)  :    initial landmarks
%
%   Outputs:    - init_state        :    struct containing the variables of 
%                                        the initial state
% ------------------------------------------------------------------------

init_state.P = [];
init_state.X = [];
if nargin == 2
    init_keypoints = init_keypoints.Location.';
    assert(size(init_keypoints,1) == 2);
    assert(size(init_landmarks,1) == 3);
    init_state.P = init_keypoints;
    init_state.X = init_landmarks;
end
init_state.C = [];
init_state.F = [];
init_state.T = [];
end