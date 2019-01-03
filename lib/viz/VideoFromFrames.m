




workingDir = 'plotFrames'
imageNames = dir(fullfile('plotFrames','*.jpg'));
imageNames = {imageNames.name}';

%you can change the video file name here:
outputVideo = VideoWriter(fullfile('videoPlot.avi'));

%you can change the desired framerate here:
outputVideo.FrameRate = 1;

open(outputVideo)

for ii = 1:length(imageNames)
   img = imread(fullfile('plotFrames',imageNames{ii}));
   writeVideo(outputVideo,img)
end
close(outputVideo)