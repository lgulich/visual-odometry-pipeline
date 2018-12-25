function [matched_kp_1, matched_kp_2] = getMatchedKeypoints(des_1, des_2, kp_1, kp_2)
% Get matching keypoints between two images
%
%   :param des_1: class binaryFeatures, descriptors from first image
%   :param des_2: class binaryFeatures, descriptors from second image
%   :param kp_1: class cornerPoint, keypoints from first image
%   :param kp_2: class cornerPoint, keypoints from second image
%
%   :return matched_kp_1: class cornerPoints etc, the matched keypoints from image 1
%   :return matched_kp_2: class cornerPoints etc, the matched keypoints from image 2

% match features between descriptors
index_pairs = matchFeatures(des_1, des_2);

% get the valid keypoints from matches
matched_kp_1 = kp_1(index_pairs(:,1),:);
matched_kp_2 = kp_2(index_pairs(:,2),:);
end

