% Add folders to path
thisfile = which(mfilename);
mainfolder = fileparts(thisfile);
cd(mainfolder);
addpath(genpath(mainfolder));
[status, msg, msgID] = mkdir('plotFrames');
clear thisfile mainfolder status msg msgID;
