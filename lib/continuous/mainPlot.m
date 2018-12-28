function [] = mainPlot(curr_state, T_W_C_curr, image, num_landmarks_tracked,T_W_C_all,ground_truth)

    fig=figure(1);
%%  top left: image and keypoints. +++ PLEASE CHECK IF THE SELECTED MARKS ARE
        %CORRECT (I mean .C and .X)+++
        
fig.Position = [1 1 1920 1080];
    subplot(3,4,[1,2]);
        imshow(image);
        hold on;
        scatter(curr_state.C(1,:)',curr_state.C(2,:)','rx', 'Linewidth', 2);
        scatter(curr_state.P(1,:)',curr_state.P(2,:)','gx', 'Linewidth', 2);
        legend('Candidate Key Points', 'Actual Keypoints')
%%  bottom left: number of last 20 keypoints (also pls check if num_landmarks_tracked refers to the right obj.
       
    subplot(3,4,[5,9]);
    
       plot(-19:1:0, num_landmarks_tracked); %defined in main
       title('number of tracked landmarks over last 20 frames');
       legend('number of points')
 %% bottom center: full trajectory plus ground truth
 subplot(3,4,[6,10]);
 
 if isempty(ground_truth)
            plot(T_W_C_all(1,:), T_W_C_all(3,:),'bx');
            grid off;
            axis equal;
            title('full trajectory');
            set(gcf, 'GraphicsSmoothing', 'on');
            
        else         
            history_size = size(T_W_C_all,2);
            plot(ground_truth(1:history_size,1),ground_truth(1:history_size,2),'rx',T_W_C_all(1,:), T_W_C_all(3,:), 'bx');
            grid off;
            axis equal;
            title('full trajectory');
            set(gcf, 'GraphicsSmoothing', 'on');
            legend('ground truth','vo pipeline')
 end
        

end

