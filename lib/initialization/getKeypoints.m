function [points] = getKeypoints(img, mode, params)
%   get keypoints of image
%   :param img matrix, image to extract keypoints
%   :param mode string, mode of keypoint extraction eg 'harris', 'surf',
%   'fast'
%   :return kp, matrix 2xN, the N found keypoints

% extract features according to method requested
if strcmp(mode, 'harris')
    points = detectHarrisFeatures(img);
    
elseif strcmp(mode, 'surf')
    points = detectSURFFeatures(img)
    
elseif strcmp(mode, 'FAST')
    points = detectFASTFeatures(img)
    
end

%kp = points.selectStrongest(params.n_keypoints);
end

