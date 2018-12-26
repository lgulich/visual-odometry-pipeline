function [descriptors, valid_points] = getDescriptors(img, points, params)
% Get the descriptors of an image and its keypoints
%
%   :param img: matrix, image to extract keypoints
%   :param points: class cornerPoints etc, detected keypoints
%   :param params: struct, the parameter struct
%
%   :return descriptors: class binaryFeatures, the descriptors
%   :return valid_points: class cornerPoints etc, the keypoints valid for
%   extracting a descriptor

% extract descriptor, type of descriptor is chosing according to type of
% keypoint class
[descriptors, valid_points] = extractFeatures(img, points);

end
