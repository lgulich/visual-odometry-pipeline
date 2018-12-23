function [params] = loadParams(dataset)
% savel all parameters to a parameters struct
%   :param dataset string, the dataset used, eg. 'kitti', 'malaga',
%   'parking'
%   :return params struct, the parameters

%% General parameters
    params.kitti_path = 'data/kitti';
    params.malaga_path = 'data/malaga';
    params.parking_path = 'data/parking';

%% Dataset specific parameters

if strcmp(dataset, 'kitti')
% Initialization parameters
    params.bootstrap_frames = [1,3];

% Keypoint detection
    params.n_keypoints = 100;
    params.descriptor_type = 'Block';
    params.descriptor_size = 11; % has to be odd
    assert(mod(params.descriptor_size, 2)==1)
    
elseif strcmp(dataset, 'malaga')
    error('[loadParams] params for malaga not implemented')
    
elseif strcmp(dataset, 'parking')
% Initialization parameters
    params.bootstrap_frames = [1,3];

% Keypoint detection
    params.n_keypoints = 100;
    params.descriptor_type = 'Block';
    params.descriptor_size = 11; % has to be odd
    assert(mod(params.descriptor_size, 2)==1)
end
end

