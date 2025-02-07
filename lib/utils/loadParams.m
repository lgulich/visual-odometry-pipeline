function [params] = loadParams(dataset)
% save all parameters to a parameters struct
%   :param dataset: string, the dataset used, eg. 'kitti', 'malaga', 'parking',
%       'ascento'
%   :return params: struct, the parameters
%
% NOTE: add your parameters here. If the parameter is specific for a
% dataset add it again in the 'dataset specific parameters' section and
% make sure to add a comment in the default section that it was overriden.
% For an example see the parameter 'bootstrap_frames' which is overriden
% for all datasets

%% General parameters
params.viz_enabled = false;
params.warn_enabled = false;

params.kitti_path = 'data/kitti';
params.malaga_path = 'data/malaga';
params.parking_path = 'data/parking';
params.ascento_path = 'data/ascento';


%% Initialization parameters
params.bootstrap_frames = [1, 2];   % overriden for all datasets
params.ROI = [];                    % overriden for all datasets
params.uniq = false;
params.filt_size = 5;
params.max_ratio = 0.5;   

% keypoint detection and maching
params.keypoint_type = 'harris';
params.feature_quality = 1e-4;
params.n_keypoints = 200;
params.descriptor_size = 11;        % has to be odd
assert(mod(params.descriptor_size, 2)==1)
params.matching_mode = 'patch_matching'; %'patch_matching' or 'klt'

%% Dataset specific parameters
%% params for KITTI
if strcmp(dataset, 'kitti')
    params.bootstrap_frames = [1,3];
    
    % Matching keypoints
    params.matching_mode = 'klt';
    params.n_keypoints = 1100;
    params.uniq = true;
    params.max_ratio = 0.97;
    params.feature_quality = 1e-4;
    params.filt_size = 3;
    
    % 8 point algorithm
    params.eightp_num_trials = 64000;
    params.eightp_dist_threshold = 0.001;
    params.eightp_confidence = 99.99;
    
    % Continuous operation parameters
    params.min_angle = 2;               % minimum angle for triangulating
    params.new_cand_kpt_threshold = 3;  % threshold for selecting new 
                                        % candidate keypoints
    % KLT parameters
    params.lambda = 2;                  % maximum bidirectional error
    params.num_pyr_levels = 6;
    params.bl_size = [31, 31];
    params.max_its = 40;

    % P3P parameters
    params.max_num_trials = 1000;
    params.conf = 99.99;
    params.max_repr_err = 1;
    
    % Triangulation of new landmarks parameters
    params.strong_to_uniform_kp_ratio = 0.13;
    
%% params for MALAGA
elseif strcmp(dataset, 'malaga')
    params.bootstrap_frames = [1,3];
    
    % Matching keypoints
    params.n_keypoints = 1000;
    params.uniq = true;
    params.max_ratio = 0.9;
    params.feature_quality = 1e-4;
    params.filt_size = 3;
    
    % 8 point algorithm
    params.eightp_num_trials = 64000;
    params.eightp_dist_threshold = 0.001;
    params.eightp_confidence = 99.99;
    
    % Continuous operation parameters
    params.min_angle = 2;               % minimum angle for triangulating
    params.new_cand_kpt_threshold = 3;  % threshold for selecting new 
                                        % candidate keypoints
    % KLT parameters
    params.lambda = 2;                  % maximum bidirectional error
    params.num_pyr_levels = 5;
    params.bl_size = [31, 31];
    params.max_its = 40;

    % P3P parameters
    params.max_num_trials = 1000;
    params.conf = 99.99;
    params.max_repr_err = 1;
    
    % triangulation of new landmarks parameters
    params.strong_to_uniform_kp_ratio = 0.16;

%% params for PARKING
elseif strcmp(dataset, 'parking')
    params.bootstrap_frames = [1,5];
    
    % Matching keypoints
    params.n_keypoints = 200;
    params.uniq = true;
    params.max_ratio = 0.9;
    
    % 8 point algorithm
    params.eightp_num_trials = 32000;
    params.eightp_dist_threshold = 0.0001;
    params.eightp_confidence = 81;
    
    % Continuous operation parameters
    params.min_angle = 2;               % minimum angle for triangulating
    params.new_cand_kpt_threshold = 3;  % threshold for selecting new 
                                        % candidate keypoints
    % KLT parameters
    params.lambda = 0.56;                  % maximum bidirectional error
    params.num_pyr_levels = 4;
    params.bl_size = [23, 23];
    params.max_its = 32;

    % P3P parameters
    params.max_num_trials = 16000;
    params.conf = 99.973;
    params.max_repr_err = 0.6345;
    
    % triangulation of new landmarks parameters
    params.strong_to_uniform_kp_ratio = 0.08;
    
%% params for ASCENTO
elseif strcmp(dataset, 'ascento')
    params.bootstrap_frames = [1,5];
    
    % Rectangular region for detecting Harris corners
    params.ROI = [62.5 81.5 618 329];        % larger       
    % params.ROI = [92.5 99.5 571 293];        % smaller
 
    % 8 point algorithm
    params.eightp_num_trials = 32000;
    params.eightp_dist_threshold = 0.0001;
    params.eightp_confidence = 85;
    
    % Continuous operation parameters
    params.min_angle = 2;               % minimum angle for triangulating
    params.new_cand_kpt_threshold = 3;  % threshold for selecting new 
                                        % candidate keypoints
    % KLT parameters
    params.lambda = 1;                  % maximum bidirectional error
    params.num_pyr_levels = 5;
    params.bl_size = [31, 31];
    params.max_its = 32;

    % P3P parameters
    params.max_num_trials = 32000;
    params.conf = 85.0;
    params.max_repr_err = 0.8;
    
    % triangulation of new landmarks parameters
    params.strong_to_uniform_kp_ratio = 0.1;
end

end
