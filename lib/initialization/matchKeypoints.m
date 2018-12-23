function [matched_kp_1, matched_kp_2] = getMatchedKeypoints(des_1, des_2, kp_1, kp_2)
% get matching keypoints between two images
%   :param des_1 matrix MxN, N descriptors from first image
%   :param des_2 matrix MxN, N descriptors from second image
%   :return matched_kp_1 vector 2xN, the N matched keypoints from image 1
%   :return matched_kp_2 vector 2xN, the N matched keypoints from image 2

% match features between descriptors
indexPairs = matchFeatures(des_1, des_2);

% get the valid keypoints from matches
matched_kp_1 = kp_1(indexPairs(:,1),:);
matched_kp_2 = kp_2(indexPairs(:,2),:);
end

