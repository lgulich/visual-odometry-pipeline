
clear all
close all
clc

for i = range
    
K = [483.4384512521193, 0,       369.22366773561635;
    0,      483.4384512521193, 252.57930034463308;
    0,      0,       1];

dist_coeffs = [-0.0409132294348775, 0.003052350398358245, 0.0076469517511512895, -0.013178638115714144];

params.cam = cameraParameters('IntrinsicMatrix', K.', 'RadialDistortion', dist_coeffs(1:2), 'TangentialDistortion', dist_coeffs(3:4));

img0 = imread(['data/ascento' sprintf('/images/img_%05d.png',1)]);
img1 = imread(['data/ascento' sprintf('/images/img_%05d.png',3)]);
img0_rect = undistortImage(img0, params.cam);

subplot(1,2,1)
imshow(img0)
subplot(1,2,2)
imshow(img0_rect)

end