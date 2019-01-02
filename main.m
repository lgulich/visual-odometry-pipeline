%% Clear Workspace
clear
% TODO VK
% close all
% TODO VK
clc
rng(1) % set seed for repeatable results

%% Setup
ds = 3; % dataset: 0: KITTI, 1: Malaga, 2: parking, 3: ascento
af = 1; % additional feature: 0: off, 1:on

if af==1 && ds~=3
    error('Error: Additional feature not possible with the selected dataset.')
end

datasets={'kitti', 'malaga', 'parking', 'ascento'};
ground_truth = [];

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
    last_frame = 999;
    K = load([ascento_path '/K.txt']);
    load([ascento_path '/est_states.mat']);
    if af == 1
        est_states_start_index = 18;
    end
else
    error('dataset not found');
    
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
        sprintf('/images_rect/img_%05d.png',bootstrap_frames(1))]);
    img1 = imread([ascento_path ...
        sprintf('/images_rect/img_%05d.png',bootstrap_frames(2))]);
    
else
    error('dataset not found');
    
end

[init_pose, init_keypoints, init_landmarks] = initialize(img0, img1, params);
display(init_pose);
%% Initialize plot
num_tracked_landmarks_all = [zeros(1,18), size(init_landmarks,2), ...
    zeros(1,last_frame-bootstrap_frames(2))];

% Only take the x and z coordinates of the translation vector
t_W_C_all = [zeros(2,18), init_pose([1,3],end), ...
    zeros(2,last_frame-bootstrap_frames(2))];

% Each cell contains the x and z coordinates of the tracked landmarks of
% the corresponding frames
trackedLandmarksOverLast20Frames = cell(1,20);
for i = 1:19
    trackedLandmarksOverLast20Frames{i} = single([]);
end
trackedLandmarksOverLast20Frames{end} = init_landmarks([1,3],:);
last20FramesIdx = 1:20;

%% Continuous operation
clf;
fprintf('\n Press any key to start the continous operation...');
% pause; % TODO remove before hand-in

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
            sprintf('/images_rect/img_%05d.png',i)]));
        
    else
        error('dataset not found');
        
    end
    
    % Update state and camera pose
    %     [curr_state, T_W_C_curr] = processFrame(image, prev_img, prev_state, params);
    %TODO VK
    curr_state = prev_state;
    T_W_C_curr = zeros(3,4);
    
    if af == 1
        if i > range(1)
            
            % update vo state
            
            % update wo state
            x_curr = double((-ascento_est_states{i*20-2}.EstRobotYPos) - x_initial);
            z_curr = double(ascento_est_states{i*20-2}.EstRobotXPos - z_initial);
            gamma_curr = double(ascento_est_states{i*20-2}.EstRobotGammaOrient - gamma_initial);
            theta_curr = double(double(ascento_est_states{i*20-2}.EstThetaMean));
            
            robot_pose_wo_curr(1) = x_curr*cos(gamma_initial)+z_curr*sin(gamma_initial);
            robot_pose_wo_curr(2) = z_curr*cos(gamma_initial)-x_curr*sin(gamma_initial);
            robot_pose_wo_curr(3) = gamma_curr;
            robot_pose_wo_curr(4) = theta_curr;
            
            % get translation vector from vo
            % d_robot_pose_vo = robot_pose_vo_curr - robot_pose_vo_last;
            
            % get translation vector from wo
            d_robot_pose_wo = robot_pose_wo_curr - robot_pose_wo_last;
            
            % combine translation vectors
            % [d_robot_pose_W, kalman_state] = estimate_d_robot_pose_W(d_robot_pose_vo, d_robot_pose_wo, kalman_state);
            
            % update W state
            % robot_pose_W_curr = integrateRobotPose(robot_pose_W_last, d_robot_pose_W, theta_curr);
            
            % plot vo, wo and W robot poses
            if(~mod(i,15))
                %   patch_bool = plotRobotPose(robot_pose_vo_curr, 'c', image, patch_bool));
                patch_bool = plotRobotPose(robot_pose_wo_curr, 'r', image, patch_bool);
                %   patch_bool = plotRobotPose(robot_pose_W_curr, 'r', image, patch_bool));
            end
            
            % update vo iterators
            %             robot_pose_vo_last = robot_pose_vo_curr;
            
            % update wo iterators
            robot_pose_wo_last = robot_pose_wo_curr;
            
            % update W iterators
            %             robot_pose_W_last = robot_pose_W_curr;
            
        elseif i == range(1)
            
            % initialize W iterators
            theta_last = double(ascento_est_states{i*20-2}.EstThetaMean);
            
            robot_pose_W_last(1)= 0.0;
            robot_pose_W_last(2)= 0.0;
            robot_pose_W_last(3)= 0.0;
            robot_pose_W_last(4) = theta_last;
            
            % initialialize vo iterators
            theta_last = double(ascento_est_states{i*20-2}.EstThetaMean);
            T_W_C_last = T_W_C_curr;
            
            robot_pose_vo_last(1)= 0.0;
            robot_pose_vo_last(2)= 0.0;
            robot_pose_vo_last(3)= 0.0;
            robot_pose_vo_last(4) = theta_last;
            
            % initialize wo iterators
            robot_pose_wo_last(1)= 0.0;
            robot_pose_wo_last(2)= 0.0;
            robot_pose_wo_last(3)= 0.0;
            robot_pose_wo_last(4) = theta_last;
            
            x_initial = double(-ascento_est_states{range(1)*20-2}.EstRobotYPos);
            z_initial = double(ascento_est_states{range(1)*20-2}.EstRobotXPos);
            gamma_initial = double(ascento_est_states{range(1)*20-2}.EstRobotGammaOrient);
            
            % initialize plot
            patch_bool = true; 
            
        end
    end
    
    
    % Plot
    %     [last20FramesIdx, ...
    %      num_tracked_landmarks_all, t_W_C_all, trackedLandmarksOverLast20Frames] = ...
    %                                 plotVO( curr_state, T_W_C_curr, image, ...
    %                                 num_tracked_landmarks_all, t_W_C_all, ...
    %                                 trackedLandmarksOverLast20Frames, ...
    %                                 last20FramesIdx, ground_truth );
    
    % Make sure that plots refresh
    pause(0.01);
    
    prev_img = image;
    prev_state = curr_state;
end
