function saveVideo(F,filename)
testtime = datestr(now,'mm-dd-yyyy HH-MM-SS');
    video_filename = sprintf('%s-%s', [filename,testtime]);
    % % % create the video writer with 30 fps
      writerObj = VideoWriter(video_filename,'MPEG-4');
      writerObj.FrameRate = 30;
    % set the seconds per image
    % open the video writer
    open(writerObj);
    % write the frames to the video
    for i= 2:length(F)
        % convert the image to a frame
        frame = F(i);
        writeVideo(writerObj, frame);
    end
    % close the writer object
    close(writerObj);
end
