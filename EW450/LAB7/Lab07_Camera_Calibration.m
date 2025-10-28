%% Camera Initialization 

[cam,prv,handles] = initCamera;

%% Adjust Camera Settings

camSettings = adjustCamera(cam);

%% Acquire Calibration Images

imBaseName = 'im';
calFolderName = 'Lab07_CamCal';
nImages = 10;
getCalibrationImages(prv,imBaseName,calFolderName,nImages);

%% Calibration

cameraCalibrator;

%% Defining Used Calibration Images (Adjust as needed --> check GUI)

imagesUsed = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10];

%% Parse Intrinsic Matrix

A_c2m = cameraParams.IntrinsicMatrix.'; % <-- Note the transpose

%% Parse Extrinsic Matrices

n = cameraParams.NumPatterns; % <-- Total number of calibration images
for i = 1:n
    R_f2c = cameraParams.RotationMatrices(:,:,i).'; % <-- Note the transpose
    d_f2c = cameraParams.TranslationVectors(i,:).'; % <-- Note the transpose
    H_f2c{i} = [R_f2c, d_f2c; 0,0,0,1]; % <-- Each extrinsic matrix is contained in a cell
end

%% Recover Body-fixed Fiducial Points

p_f = cameraParams.WorldPoints.'; % Parse x/y fiducial coordinates (note the transpose)
p_f(3,:) = 0; % Fiducial z-coordinates (fiducial is 2D, so z is 0)
p_f(4,:) = 1; % Make points homogeneous

%% Reproject Body-fixed Fiducial Points

for i = 1:n
    % Define filename
    imName = sprintf('%s%03d.png',imBaseName,imagesUsed(i));
    % Load image
    im = imread( fullfile(calFolderName,imName) );
    % Plot image
    fig = figure('Name',imName);
    axs = axes('Parent',fig);
    img = imshow(im,'Parent',axs);
    hold(axs,'on');
    % Calculate projection matrix
    P_f2m = A_c2m * H_f2c{i}(1:3,:);
    % Project points using intrinsics and extrinsics
    tilde_p_m = P_f2m*p_f; % <-- Scaled pixel coordinates
    p_m = tilde_p_m./tilde_p_m(3,:); % <-- Pixel coordinates
    % Plot points
    % - Fiducial origin point
    plt0 = plot(axs,p_m(1,1),p_m(2,1),'ys','LineWidth',3,'MarkerSize',8);
    % - All other fiducial points
    plti = plot(axs,p_m(1,2:end),p_m(2,2:end),'go','LineWidth',2,'MarkerSize',8);
    % Label points
    for j = 1:size(p_m,2)
        txt(j) = text(axs,p_m(1,j),p_m(2,j),sprintf('$p_{%d}^{m}$',j), ...
            'Interpreter','Latex','Color','m','FontSize',14);
    end
end

%% Projecting 3D Computer Graphics into Images

% Chose image index
i = 10;

% Open Wall-E visualization and get the figure handle
% DO NOT MANUALLY CLOSE THIS FIGURE
figWallE = open('Lab07_Wall-E.fig');

% Recover the patch object from the Wall-E visualization figure handle
ptc_b = findobj(figWallE,'Type','patch','Tag','Wall-E');

p_b = ptc_b.Vertices.'; % <-- Note the transpose
p_b(4,:) = 1; % Make points homogeneous positions

% Define checkerboard info
[boardSize,squareSize] = checkerboardPoints2boardSize( cameraParams.WorldPoints );

% Define Frame o relative to the fiducial frame f
x_o2f = squareSize*(boardSize(2)-2)/2; % x-offset of frame o wrt frame f
y_o2f = squareSize*(boardSize(1)-2)/2; % y-offset of frame o wrt frame f
H_o2f = Tx(x_o2f)*Ty(y_o2f)*Rx(pi);

% Define Wall-E's body-fixed frame relative to frame o.
% Note: We will initially use the identity, but we can change this frame
% if we want Wall-E to drive.
H_b2o = eye(4);

% Define applicable extrinsics
H_b2c = H_f2c{i}*H_o2f*H_b2o;
% Define projection matrix
P_b2m = A_c2m * H_b2c(1:3,:);

% Project points using intrinsics and extrinsics
tilde_p_m = P_b2m*p_b; % <-- Scaled pixel coordinates
p_m = tilde_p_m./tilde_p_m(3,:); % <-- Pixel coordinates

% Define filename
imName = sprintf('%s%03d.png',imBaseName,imagesUsed(i));
% Load image
im = imread( fullfile(calFolderName,imName) );
% Plot image
fig = figure('Name',imName);
axs = axes('Parent',fig);
img = imshow(im,'Parent',axs);
hold(axs,'on');

% Place Wall-E in the image
ptc_m = copyobj(ptc_b(1),axs); % Copy the Wall-E patch object
ptc_m.Vertices = p_m(1:2,:).'; % Updated the vertices with pixel coordinates

%% Improving the Visualization

% Recover the patch object from the Wall-E visualization figure handle
ptc_b = findobj(figWallE,'Type','patch','Tag','Wall-E');

p_b = ptc_b.Vertices.'; % <-- Note the transpose
% NOTE: We will not make these points homogeneous, so ``projectWithFalseDepth''
% will run a little faster

% Define checkerboard info
[boardSize,squareSize] = checkerboardPoints2boardSize( cameraParams.WorldPoints );
% Define Frame o relative to the fiducial frame f
x_o2f = squareSize*(boardSize(2)-2)/2; % x-offset of frame o wrt frame f
y_o2f = squareSize*(boardSize(1)-2)/2; % y-offset of frame o wrt frame f
H_o2f = Tx(x_o2f)*Ty(y_o2f)*Rx(pi);

% Define Wall-E's body-fixed frame relative to frame o.
% Note: We will initially use the identity, but we can change this frame
% if we want Wall-E to drive.
H_b2o = eye(4);

% Define applicable extrinsics
H_b2c = H_f2c{i}*H_o2f*H_b2o;
% Projection matrix
P_b2m = A_c2m * H_b2c(1:3,:);

% Project points with added false depth
p_m_falseDepth = projectWithFalseDepth(p_b,P_b2m);

% Define filename
imName = sprintf('%s%03d.png',imBaseName,imagesUsed(i));
% Load image
im = imread( fullfile(calFolderName,imName) );
% Plot image
fig = figure('Name',imName);
axs = axes('Parent',fig);
img = imshow(im,'Parent',axs);
hold(axs,'on');

% Add a light to the scene
addSingleLight(axs);

% Place Wall-E in the image
ptc_m = copyobj(ptc_b(1),axs); % Copy the Wall-E patch object
ptc_m.Vertices = p_m_falseDepth.'; % Updated the vertices with false depth pixel coordinates

%% Animating the Improved Visualization

% Define position information
d_b2o = []; x_hat = [];
r = squareSize*min(boardSize)/2;
phi = linspace(0,2*pi,50);
d_b2o(1,:) = r*cos(phi); % x-position
d_b2o(2,:) = r*sin(phi); % y-position
d_b2o(3,:) = 0; % z-position

% Define orientation information
z_hat = [0;0;1]; % z-direction

% x-direction
x_hat(1,:) = -sin(phi);
x_hat(2,:) = cos(phi);
x_hat(3,:) = 0;

% Define rigid body transforms
H_b2o = {};
for k = 1:numel(phi)
    % Define Wall-E's pose relative to the center frame
    y_hat = cross(z_hat,x_hat(:,k));

    % Define orientation
    R_b2o = [x_hat(:,k),y_hat,z_hat];

    % Define pose
    H_b2o{k} = [R_b2o, d_b2o(:,k); 0,0,0,1];
end

% Recover the patch object from the Wall-E visualization figure handle
ptc_b = findobj(figWallE,'Type','patch','Tag','Wall-E');

% Define the vertices of Wall-E relative to the body-fixed frame
p_b = ptc_b.Vertices.'; % <-- Note the transpose

% NOTE: We will not make these points homogeneous, so ``projectWithFalseDepth''
% will run a little faster
% Define image
i = 5;

% Initialize an MPEG-4 video writer object
vid = VideoWriter('Lab07_WallE_Driving.mp4','MPEG-4');
% Open video writer object
open(vid);

% Define filename
imName = sprintf('%s%03d.png',imBaseName,imagesUsed(i));

% Load image
im = imread( fullfile(calFolderName,imName) );

% Plot image
fig = figure('Name',imName);
axs = axes('Parent',fig);
img = imshow(im,'Parent',axs);
hold(axs,'on');
addSingleLight(axs);
ptc_m = copyobj(ptc_b(1),axs);
set(fig,'Color',[1,1,1]); % set to white background

for k = 1:numel(H_b2o)
    % Define applicable extrinsics
    H_b2c = H_f2c{i}*H_o2f*H_b2o{k};

    % Define projection
    P_b2m = A_c2m * H_b2c(1:3,:);

    % Project points with added false depth
    p_m_falseDepth = projectWithFalseDepth(p_b,P_b2m);

    % Update Wall-E
    ptc_m.Vertices = p_m_falseDepth.';
    drawnow
    
    % "Take a picture" of the axes handle
    % (this assumes "axs" is the axes object you want in the video)
    frame = getframe(axs);
    % Write the frame to the video
    writeVideo(vid,frame);
end

% Close video writer object
close(vid);

%% Creating “Real-Time” Visualization

% Recover the patch object from the Wall-E visualization figure handle
ptc_b = findobj(figWallE,'Type','patch','Tag','Wall-E');

% Recover the patch object from the Wall-E visualization figure handle
p_b = ptc_b.Vertices.'; % <-- Note the transpose
% Do not make the points homogenous. This will make projectWithFalseDepth
% run a little faster.

% Define Wall-E's body-fixed frame relative to frame o.
% Note: We will initially use the identity, but we can change this frame
% if we want Wall-E to drive.
H_b2o = eye(4);

% Create a figure for the real-time example
im = get(prv,'CData'); % Get a new image from the camera
figRealTime = figure('Name','"Real-time Example"'); % Create a new figure
axsRealTime = axes('Parent',figRealTime); % Create an axes in the figure
imgRealTime = imshow(im,'Parent',axsRealTime); % Show the image in the axes
hold(axsRealTime,'on');
addSingleLight(axsRealTime);
ptc_m = copyobj(ptc_b,axsRealTime);

% Define square size for checkerboard fiducial
squareSize = 10.00; % <-- Confirm that this is correct for your checkerboard

% Create a loop to recover extrinsics
while ishandle(figRealTime)
    % Allow preview and figure(s) to update
    drawnow
    
    % Get the image from the preview
    im = get(prv,'CData');
    % Define p m from the image of the checkerboard
    % NOTE: MATLAB's definition of "imagePoints" relates to p m as follows:
    % % Define "imagePoints" from p m
    % imagePoints = p m(1:2,:).'; % <-- Note the transpose
    % % Define p m from "imagePoints"
    % p m(1:2,:) = imagePoints.'; % <-- Note the transpose
    % p m(3,:) = 1; % <-- Convert to homogeneous, 2D
    % pixel position
    [imagePoints,boardSize] = detectCheckerboardPoints(im);
    
    % Define Frame o relative to the fiducial frame f
    x_o2f = squareSize*(boardSize(2)-2)/2; % x-offset of frame o wrt frame f
    y_o2f = squareSize*(boardSize(1)-2)/2; % y-offset of frame o wrt frame f
    H_o2f = Tx(x_o2f)*Ty(y_o2f)*Rx(pi);
    
    % Check if full checkerboard was detected
    if any(~isfinite(imagePoints),'all') || any(boardSize == 0)
        % One or more checkerboard points are not tracked
        % Hide Wall-E
        set(ptc_m,'Visible','off');
        % Continue to the next iteration of the while-loop
        continue
    else
        % Checkerboard is tracked
        % Show Wall-E
        set(ptc_m,'Visible','on');
    end
    
    % Define p f given "boardSize" and "squareSize"
    % NOTE: MATLAB's definition of "worldPoints" relates to p f as follows:
    % % Define "worldPoints" from p f
    % worldPoints = p f(1:2,:).'; % <-- Note the transpose
    % % Define p f from "worldPoints"
    % p f(1:2,:) = worldPoints.'; % <-- Note the transpose
    % p f(3,:) = 0; % <-- Define z-coordinate
    % p f(4,:) = 1; % <-- Convert to homogeneous, 3D
    % % coordinate relative to the
    % % fiducial frame
    [worldPoints] = generateCheckerboardPoints(boardSize,squareSize);
    
    % Recover the checkerboard pose relative to the camera frame (H f2c)
    [R_c2f, tpose_d_f2c] = extrinsics(...
    imagePoints,worldPoints,cameraParams);
    R_f2c = R_c2f.';
    d_f2c = tpose_d_f2c.';
    H_f2c = [R_f2c, d_f2c; 0,0,0,1];
    
    % Define applicable extrinsics
    H_b2c = H_f2c*H_o2f*H_b2o;
    
    % Define projection
    P_b2m = A_c2m * H_b2c(1:3,:);
    
    % Project points with added false depth
    p_m_falseDepth = projectWithFalseDepth(p_b,P_b2m);
    
    % Update Image
    set(imgRealTime,'CData',im);
    
    % Update Wall-E
    ptc_m.Vertices = p_m_falseDepth.';
    drawnow;
end