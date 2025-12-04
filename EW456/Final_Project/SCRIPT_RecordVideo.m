%% Open Camera using known name and resolution of HP208 cameras
[cam,prv] = initCamera('Jabra PanaCast','MJPG_3840x1088');
camSettings = adjustCamera(cam);

% Note: close the camera settings user interface window when ready to
% record a video

%% Create a videowriter object to save frames to

vidObj = VideoWriter('Star.mp4','MPEG-4');
open(vidObj);

%% Record frames 
count = 1; % frame counter
while true
    frm = get(prv,'CData');
    writeVideo(vidObj,frm);
    drawnow;
    pause(1/15);
    disp(['Frames Written: ' num2str(count) ', Press ctrl+c in command window to stop'])
    count = count+1;
end

%% Save video MATLAB current working directory
close(vidObj);

