%% Clear Workspace
clear all
close all
clc
rng(1) % set seed for repeatable results

%% Setup
ds = 0; % 0: KITTI, 1: Malaga, 2: parking, 3: ascento
datasets={'kitti', 'malaga', 'parking', 'ascento'};

% load params
params = loadParams(datasets{ds+1});

kitti_path = params.kitti_path;
malaga_path = params.malaga_path;
parking_path = params.parking_path;
ascento_path = params.ascento_path;


if ds == 0
    % need to set kitti_path to folder containing "00" and "poses"
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];

elseif ds == 1
    % Path containing the many files of Malaga 7.
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];

elseif ds == 2
    % Path containing images, depths and all...
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);

    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);

elseif ds == 3
    % Path containing images, depths and all...
    assert(exist('ascento_path', 'var') ~= 0);
    last_frame = 200;
    K = load([ascento_path '/K.txt']);
    load([ascento_path '/est_states.mat']);

else
    assert(false);

end

% set camera parameters (! matlab uses transposed K matrix)
params.cam = cameraParameters('IntrinsicMatrix', K.');

%% Bootstrap
% frames used for initial bootstrapping
bootstrap_frames = params.bootstrap_frames;

if ds == 0
    img0 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);

elseif ds == 1
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));

elseif ds == 2
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));

elseif ds == 3
    img0 = imread([ascento_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]);
    img1 = imread([ascento_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]);

else
    assert(false);

end

[init_pose, init_keypoints, init_landmarks] = initialize(img0, img1, params);

% initialize plot
num_landmarks_tracked = [zeros(1,19), length(init_landmarks)];
T_W_C_all_vec = zeros(3,1);
T_W_C_all_hom_old = eye(4);
T_W_C_hom_old = eye(4);

%% Continuous operation

% fprintf('\n Press any key to start the continous operation...');
% pause; %% TODO remove before hand-in

% Setup
range = (bootstrap_frames(2)+1):last_frame;
prev_img = img1;
prev_state = initializeState(init_keypoints, init_landmarks);

% Process frames
for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        image = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);

    elseif ds == 1
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));

    elseif ds == 3
        image = im2uint8(imread([ascento_path ...
            sprintf('/images/img_%05d.png',i)]));

    else
        assert(false);

    end

    % Update state and camera pose
    [curr_state, T_W_C_curr] = processFrame(image, prev_img, prev_state, params);

    % Make sure that plots refresh
    pause(0.01);
    
    % PLOT
    num_landmarks_tracked = [num_landmarks_tracked(1,2:end), length(curr_state.X)];
    
    T_W_C_curr_hom_new = T_W_C_hom_old * [T_W_C_curr ; 0 0 0 1];
    
    T_W_C_vec = T_W_C_curr_hom_new(1:3,4);
    
    
    T_W_C_all_vec=[T_W_C_all_vec, T_W_C_vec];
    
    mainPlot(curr_state, T_W_C_curr, image, num_landmarks_tracked,T_W_C_all_vec,ground_truth);
    
    %END PLOT
    
    prev_img = image;
    prev_state = curr_state;
end



% set camera parameters (! matlab uses transposed K matrix)
params.cam = cameraParameters('IntrinsicMatrix', K.');

%% Bootstrap
% frames used for initial bootstrapping
bootstrap_frames = params.bootstrap_frames;

if ds == 0
    img0 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);

elseif ds == 1
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));

elseif ds == 2
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));

elseif ds == 3
    img0 = imread([ascento_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]);
    img1 = imread([ascento_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]);

else
    assert(false);

end

[init_pose, init_keypoints, init_landmarks] = initialize(img0, img1, params);

% initialize plot
num_landmarks_tracked = [zeros(1,19), length(init_landmarks)];
T_W_C_all=zeros(3,1);


%% Continuous operation

% fprintf('\n Press any key to start the continous operation...');
% pause; %% TODO remove before hand-in

% Setup
range = (bootstrap_frames(2)+1):last_frame;
prev_img = img1;
prev_state = initializeState(init_keypoints, init_landmarks);

% Process frames
for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        image = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);

    elseif ds == 1
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));

    elseif ds == 3
        image = im2uint8(imread([ascento_path ...
            sprintf('/images/img_%05d.png',i)]));

    else
        assert(false);

    end

    % Update state and camera pose
    [curr_state, T_W_C_curr] = processFrame(image, prev_img, prev_state, params);

    % Make sure that plots refresh
    pause(0.01);
    
    % PLOT
    num_landmarks_tracked = [num_landmarks_tracked(1,2:end), length(curr_state.X)];
    
    T_W_C_curr_hom_new = T_W_C_hom_old * [T_W_C_curr ; 0 0 0 1];
    
    T_W_C_vec = T_W_C_curr_hom_new(1:3,4);
    
    
    T_W_C_all_vec=[T_W_C_all_vec, T_W_C_vec];
    
    mainPlot(curr_state, T_W_C_curr, image, num_landmarks_tracked,T_W_C_all_vec,ground_truth);
    
    %END PLOT
    
    prev_img = image;
    prev_state = curr_state;
end
