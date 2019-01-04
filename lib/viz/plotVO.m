function [last20FramesIdx_next, num_tracked_landmarks, t_W_C_all, ...
        trackedLandmarksOverLast20Frames] = plotVO( ...
    curr_state, T_W_C_curr, image, num_tracked_landmarks, t_W_C_all, ...
            trackedLandmarksOverLast20Frames, last20FramesIdx, ground_truth)

% Update plot data
curr_frame_idx = last20FramesIdx(end);
num_tracked_landmarks(curr_frame_idx) = size(curr_state.X,2);
t_W_C_all(:,curr_frame_idx) = T_W_C_curr([1,3],end);
% trackedLandmarksOverLast20Frames = circshift(trackedLandmarksOverLast20Frames,-1);
trackedLandmarksOverLast20Frames{end} = curr_state.X([1,3],:);

fig = figure(1);
fig.Position = [1 1 1920 1080];

% Top left plot
subplot(4,4,[1,2,5,6])
imshow(image), hold on
plot(curr_state.C(1,:)',curr_state.C(2,:)','rx'), hold on
plot(curr_state.P(1,:)',curr_state.P(2,:)','gx'), hold off
title('Current image.')
legend({'Candidate Keypoints $\quad\quad$', 'Actual Keypoints $\quad\quad$'}, 'FontSize', 8, ...
                                'Interpreter', 'latex')

% Bottom left
subplot(4,4,[9,13])
plot(last20FramesIdx-20, num_tracked_landmarks(last20FramesIdx),  'color', 'b')
title('Number of tracked landmarks over last 20 frames.')
axis('tight')
ylim([0,200])

% Bottom right
subplot(4,4,[10,14])
plot(t_W_C_all(1,1:curr_frame_idx), t_W_C_all(2,1:curr_frame_idx), ...
    'b-o', 'MarkerSize', 2.4, 'MarkerFaceColor', 'b'), grid off, axis equal
title('Full trajectory.')
set(gcf, 'GraphicsSmoothing', 'on')

% Right

landmarks = cell2mat(trackedLandmarksOverLast20Frames);
landmarks = unique(landmarks.', 'rows').';
subplot(4,4,[3,4,7,8,11,12,15,16])

 % define limits of plot
        xlim('manual');
        ylim('manual');
%         
        r_x = 10*abs(max(t_W_C_all(1,last20FramesIdx))-min(t_W_C_all(1,last20FramesIdx)))/2+1;
        r_z = 10*abs(max(t_W_C_all(2,last20FramesIdx))-min(t_W_C_all(2,last20FramesIdx)))/2+1;
        xlim([(t_W_C_all(1,end))-r_x, (t_W_C_all(1,end))+r_x]);
        ylim([(t_W_C_all(2,end))-r_z, (t_W_C_all(2,end))+r_z]);
% 



plot(t_W_C_all(1,last20FramesIdx), t_W_C_all(2,last20FramesIdx),'b-o',...
            'MarkerSize', 1.6, 'MarkerFaceColor', 'b'), hold on

      X_avg = mean(curr_state.X,2);
        X_dev = 2*std(curr_state.X');
        x_limits = [X_avg(1)-X_dev(1), X_avg(1)+X_dev(1)];
        y_limits = [X_avg(2)-X_dev(2), X_avg(2)+X_dev(2)];
        z_limits = [X_avg(3)-X_dev(3), X_avg(3)+X_dev(3)];
        x_mask = curr_state.X(1,:)>x_limits(1) & curr_state.X(1,:)<x_limits(2);
        y_mask = curr_state.X(2,:)>y_limits(1) & curr_state.X(2,:)<y_limits(2);
        z_mask = curr_state.X(3,:)>z_limits(1) & curr_state.X(3,:)<z_limits(2);
        mask = x_mask & y_mask & z_mask;
        
       plot(curr_state.X(1, mask), curr_state.X(3, mask), 'ks', 'MarkerSize', 6,...
    'MarkerFaceColor',[0,0,0]);grid off, axis equal, hold off
% plot(landmarks(1,:), landmarks(2,:), 'kx', 'MarkerSize', 3.2, 'Color', [0.88,0.88,0.88]), hold on
% 
% hold off
% 
 title('Trajectory of last 20 frames and landmarks.')
set(gcf, 'GraphicsSmoothing', 'on')

axis equal

% Update indexes
last20FramesIdx_next = last20FramesIdx + 1;

end