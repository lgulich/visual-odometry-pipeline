%% Script to to read in images and states from bag file
% DO NOT RUN AGAIN 
% !!! Several steps must be done to do this in your computer:
% 1. Install the robotics ros tool box from robotics addons
% 2. Generate your custom message with rosgenmsg
% 3. Follow the instruction generates
% 4. Increase the Java Heap Memory Preferences

%% Specify bag params
ascento_bag_name = 'ML_3.bag';
ascento_bag_frames = 1000;
ascento_bag_hz_camera = 20; %[Hz]
ascento_bag_length = (ascento_bag_frames+1)/ascento_bag_hz_camera; %[s]

%% Read in bag file
ascento_bag = rosbag(ascento_bag_name);

%% Read in images from bag file 
ascento_bag_select_images = select(ascento_bag, 'Time', [ascento_bag.StartTime, ascento_bag.StartTime+ascento_bag_length], 'Topic', '/arduino_vi/forward/image_raw');
ascento_images = readMessages(ascento_bag_select_images);

for i=1:size(ascento_images, 1)
    imwrite(readImage(ascento_images{i}), ['data/ascento/images/', 'img_', num2str(i, '%05d'), '.png'])
end

%% Read in estimated statess
ascento_bag_select_states = select(ascento_bag, 'Time', [ascento_bag.StartTime, ascento_bag.StartTime+ascento_bag_length], 'Topic', '/debug/sara/est_states');
ascento_est_states = readMessages(ascento_bag_select_states, "DataFormat", "struct");

save('data/ascento/est_states', 'ascento_est_states');