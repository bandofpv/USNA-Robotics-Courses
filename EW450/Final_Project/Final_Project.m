%% Camera Initialization

[cam,prv,handles] = initCamera;

%% Adjusting Camera Settings

camSettings = adjustCamera(cam);

%% Calibrate Camera With Handheld

nImages = 10;
[cameraParams] = calibrateCameraWithHandheld(prv,nImages);

%% Initialize UR3e

ur = URQt('UR3e');
ur.Initialize;

%% Calibrate Camera and Robot

[H_o2c,H_c2o] = calibrateCameraAndRobot(prv,ur,cameraParams,nImages);

%% Capture Image

imDist = get(prv,'CData'); %<-- Distorted image

%% 

% Piece tags:Tag Family, Tag ID, Tag Scale, O/X
tags = {'tag36h11' 451 25.0 'O';
        'tag36h11' 452 25.0 'O';
        'tag36h11' 453 25.0 'O';
        'tag36h11' 464 25.0 'O';
        'tag36h11' 465 25.0 'O';
        'tag36h11' 461 25.0 'X';
        'tag36h11' 462 25.0 'X';
        'tag36h11' 463 25.0 'X';
        'tag36h11' 464 25.0 'X';
        'tag36h11' 464 25.0 'X'};

%% 

H_b2o = H_c2o * H_b2c; % calc pose of board frame relative to base frame

%% 

board = repmat(' ', 3, 3); % Initialize a 3x3 empty board
