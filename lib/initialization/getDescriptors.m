function [descriptors, valid_points] = getDescriptors(img, points, params)
%   get keypoints of image
%   :param img matrix, image to extract keypoints
%   :param kp matrix 2xN, N keypoints
%   :param type string, type of descriptor eg 'Block', 'FREAK', 'SURF'
%   :returns des matrix MxN, returns N descriptors, each of size M
%

%TODO add type of descriptor
[descriptors, valid_points] = extractFeatures(img, points);

end
