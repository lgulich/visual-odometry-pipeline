function  []= record()


 writerObj = VideoWriter('');
 writerObj.FrameRate = 60;
 open(writerObj);

 axis tight

 for k = 1:20 
   frame = getframe;
   writeVideo(writerObj,frame);
 end
 close(writerObj);
 