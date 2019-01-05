function [points] = getKeypoints(img, params)
% Get keypoints of specific type of image
%
%   :param img: matrix, image to extract keypoints from
%   :param params: struct, the parameter struct
%
%   :return points: class CornerPoints etc, the N found keypoints

% extract features according to the method requested
if strcmp(params.keypoint_type, 'harris')
    points = detectHarrisFeatures(img, ...
                                  'MinQuality', params.feature_quality, ...
                                  'ROI', params.ROI, ...
                                  'FilterSize', params.filt_size);
    
elseif strcmp(params.keypoint_type, 'surf')
    points = detectSURFFeatures(img)
    
elseif strcmp(params.keypoint_type, 'fast')
    points = detectFASTFeatures(img)
    
end

% select only the strongest keypoints according to metrics
points = points.selectStrongest(params.n_keypoints);
end

