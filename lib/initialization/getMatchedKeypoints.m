function [matched_kp_1, matched_kp_2] = getMatchedKeypoints(des_1, des_2, kp_1, kp_2, img_1, img_2, params)
% Get matching keypoints between two images
%
%   :param des_1: class binaryFeatures, descriptors from first image
%   :param des_2: class binaryFeatures, descriptors from second image
%   :param kp_1: class cornerPoint, keypoints from first image
%   :param kp_2: class cornerPoint, keypoints from second image
%
%   :return matched_kp_1: class cornerPoints etc, the matched keypoints 
%       from image 1
%   :return matched_kp_2: class cornerPoints etc, the matched keypoints 
%       from image 2


if strcmp(params.matching_mode, 'patch_matching')
    % match features between descriptors
    index_pairs = matchFeatures(des_1, des_2);

    % get the valid keypoints from matches
    matched_kp_1 = kp_1(index_pairs(:,1),:);
    matched_kp_2 = kp_2(index_pairs(:,2),:);


elseif strcmp(params.matching_mode, 'klt')
    % initialize a klt tracker
    tracker = vision.PointTracker('MaxBidirectionalError', params.lambda, ...
                                       'NumPyramidLevels', params.num_pyr_levels, ...
                                       'BlockSize', params.bl_size, ...
                                       'MaxIterations', params.max_its);
    initialize(tracker, kp_1.Location, img_1);

    % track keypoints from image 1 in image 2
    [kp_2, is_valid] = tracker(img_2);
    
    % filter out invalid keypoints and convert all keypoits to cornerPoints
    % object
    matched_kp_1 = kp_1(is_valid, :);
    matched_kp_2 = cornerPoints(kp_2(is_valid, :));

end
end

