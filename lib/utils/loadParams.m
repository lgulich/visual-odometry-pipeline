function [params] = loadParams(dataset)
% save all parameters to a parameters struct
%   :param dataset: string, the dataset used, eg. 'kitti', 'malaga', 'parking'
%   :return params: struct, the parameters

% Note: add your parameters here. If the parameter is specific for a
% dataset add it again in the 'dataset specific parameters' section and
% make sure to add a comment in the default section that it was overriden.
% For an example see the parameter 'bootstrap_frames' which is overriden
% for all datasets
%% General parameters
    params.viz_enabled = true;

    params.kitti_path = 'data/kitti';
    params.malaga_path = 'data/malaga';
    params.parking_path = 'data/parking';

    params.bootstrap_frames = [1, 2]; % overriden for all datasets

%% Keypoint detection and maching
    params.keypoint_type = 'harris';
    params.n_keypoints = 100;
    params.descriptor_type = 'Block';
    params.descriptor_size = 11; % has to be odd
    assert(mod(params.descriptor_size, 2)==1)

%% Continuous operation parameters
params.lambda = 0.1;                    % maximum bidirectional error
params.min_angle = 4;                   % minimum angle for triangulating  
                                        % new landmarks (degrees)
params.new_cand_kpt_threshold = 3;     % threshold for selecting new 
                                        % candidate keypoints
    
%% Dataset specific parameters

% params for KITTI
if strcmp(dataset, 'kitti')
    params.bootstrap_frames = [1,3];

% params for MALAGA
elseif strcmp(dataset, 'malaga')
    params.bootstrap_frames = [1,3];

% params for PARKING
elseif strcmp(dataset, 'parking')
    params.bootstrap_frames = [1,3];

end

end
