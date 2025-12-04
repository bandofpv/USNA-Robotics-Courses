%% Create a motion-trail video and a final time-lapse image showing true-color motion paths

% clear the workspace
clear; clc; close all;

%% ---- User parameters ----

videoFile = 'Navy.mp4';   % <-- filename of recorded video
outputVideo = 'Navy_timelapse.mp4'; % filename to write timelapse video
imageFilename = 'Navy.png'; % file anme of output image

fadeFactor = 0.99;               % trail persistence per frame (0.9–0.99 typical)
thresholdValue = 50;             % motion detection sensitivity
alpha = 0.6;                     % trail transparency on video overlay

%% ---- Read video ----
vidReader = VideoReader(videoFile);
vidWriter = VideoWriter(outputVideo, 'MPEG-4');
vidWriter.FrameRate = vidReader.FrameRate;
open(vidWriter);

framePrev = rgb2gray(readFrame(vidReader));
accumTrail = zeros([size(framePrev), 3], 'double');   % fading color trail
fullTrail  = zeros([size(framePrev), 3], 'double');   % cumulative permanent trail

% ---- Display setup ----
hFig = figure('Name', 'Motion Trail Visualization', 'NumberTitle', 'off');
ax = axes('Parent', hFig);
hImg = imshow(zeros(size(accumTrail), 'uint8'), 'Parent', ax);

%% ---- Process frames ----
while hasFrame(vidReader) && ishandle(hFig)
    % Read new frame
    frameRGB = readFrame(vidReader);
    frameProcess = im2double(frameRGB);
    frameGray = rgb2gray(frameRGB);

    % Compute frame difference
    diffImg = imabsdiff(frameGray, framePrev);
    framePrev = frameGray;

    % Threshold and clean motion mask
    motionMask = diffImg > thresholdValue;
    motionMask = imdilate(motionMask, strel('disk', 2));
    sum(sum(motionMask))
    mask3 = repmat(motionMask, [1 1 3]);

    % ---- Fade old trails ----
    accumTrail = accumTrail * fadeFactor;

    % ---- Add new color pixels where motion is detected ----
    % Important: only update where motion occurs; blend in new color
    
    % Compute brightness (luminance) for both
    brightnessNew  = 0.299*frameProcess(:,:,1) + 0.587*frameProcess(:,:,2) + 0.114*frameProcess(:,:,3);
    brightnessOld  = 0.299*fullTrail(:,:,1) + 0.587*fullTrail(:,:,2) + 0.114*fullTrail(:,:,3);

    % Keep only motion pixels
    brightnessNew(~motionMask) = 0;
    brightnessOld(~motionMask) = 0;

    % Mask where new frame is brighter
    replaceMask = brightnessNew > brightnessOld;
    replaceMask3 = repmat(replaceMask, [1 1 3]);
    
    accumTrail(replaceMask3) = frameProcess(replaceMask3);  

    % Update only those pixels (retain full color of brighter pixel)
    fullTrail(replaceMask3) = frameProcess(replaceMask3);
        
    % ---- Composite the fading trail over the live frame ----
    composite = (1 - alpha) * im2double(frameRGB) + alpha * accumTrail;

    % ---- Display and write ----
    set(hImg, 'CData', composite);
    drawnow limitrate;
    writeVideo(vidWriter, im2uint8(composite));

end

% close video to save it when complete
close(vidWriter);
disp(['Motion-trail video saved as: ', outputVideo]);


%% ---- Generate and show full time-lapse image (brightest motion color) ----

backgroundFrame = im2double(frameRGB); % last frame as background
timelapseImage = backgroundFrame;      % initialize

% Compute luminance of fullTrail and background
lumTrail = 0.299*fullTrail(:,:,1) + 0.587*fullTrail(:,:,2) + 0.114*fullTrail(:,:,3);
lumBg    = 0.299*backgroundFrame(:,:,1) + 0.587*backgroundFrame(:,:,2) + 0.114*backgroundFrame(:,:,3);

% Mask where motion trail is brighter than background
brightMask = lumTrail > lumBg;
brightMask3 = repmat(brightMask, [1 1 3]);

% Copy fullTrail color where brighter
timelapseImage(brightMask3) = fullTrail(brightMask3);

% Display
figure('Name', 'Cumulative Motion Path (True Color)', 'NumberTitle', 'off');
imshow(timelapseImage);
title('Cumulative Motion Path (Brightest Motion Colors)');

% Save image
imwrite(timelapseImage, 'motion_timelapse_color.png');
disp('Time-lapse motion image saved as: motion_timelapse_color.png');

% crop image based on pixels
cropx = [1900 3500];
cropy = [150 1088];

figure('Name', 'Cumulative Motion Path (True Color)', 'NumberTitle', 'off');
imshow(timelapseImage(cropy(1):cropy(2),cropx(1):cropx(2),:));
title('Cumulative Motion Path (True Color)');

imwrite(timelapseImage, imageFilename);
disp(['Time-lapse motion image saved as: ' imageFilename]);
