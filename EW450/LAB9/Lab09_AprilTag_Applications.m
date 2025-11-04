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

%% Identify AprilTags

imDist = get(prv,'CData'); %<-- Distorted image

%% Find AprilTag(s) in an image

[id,loc,detectedFamily] = readAprilTag(imDist);

% Plot image
fig = figure;
axs = axes('Parent',fig);
hold(axs,'on');
axis(axs,'tight');
img = imshow(imDist,'Parent',axs);

% Highlight and label AprilTags
for i = 1:numel(id)
    % Highlight AprilTag
    ptc(i) = patch(axs,'Vertices',loc(:,:,i),'Faces',1:4,...
    'EdgeColor','m','FaceColor','b','FaceAlpha',0.1);

    % Define AprilTag label
    str = sprintf('%s\nID: %03d',detectedFamily(i),id(i));

    % Label AprilTag
    ang = rad2deg( atan2(...
    loc(2,1,i) - loc(1,1,i),...
    loc(2,2,i) - loc(1,2,i)) - pi/2 );
    txt = text(axs,mean(loc(:,1,i)),mean(loc(:,2,i)),str,...
    'HorizontalAlignment','center','VerticalAlignment','middle',...
        'Rotation',ang,'FontWeight','Bold','FontSize',10,'Color','b');
end

%% Find tag36h11 AprilTag(s) in an image

tagFamily = 'tag36h11';
[id,loc,detectedFamily] = readAprilTag(imDist,tagFamily);

% Plot image
fig = figure;
axs = axes('Parent',fig);
hold(axs,'on');
axis(axs,'tight');
img = imshow(imDist,'Parent',axs);

% Highlight and label AprilTags
for i = 1:numel(id)
    % Highlight AprilTag
    ptc(i) = patch(axs,'Vertices',loc(:,:,i),'Faces',1:4,...
    'EdgeColor','m','FaceColor','b','FaceAlpha',0.1);
    
    % Define AprilTag label
    str = sprintf('%s\nID: %03d',detectedFamily(i),id(i));

    % Label AprilTag
    ang = rad2deg( atan2(...
    loc(2,1,i) - loc(1,1,i),...
    loc(2,2,i) - loc(1,2,i)) - pi/2 );
    txt = text(axs,mean(loc(:,1,i)),mean(loc(:,2,i)),str,...
    'HorizontalAlignment','center','VerticalAlignment','middle',...
        'Rotation',ang,'FontWeight','Bold','FontSize',10,'Color','b');
end

%% Find and recover pose of AprilTag

% -> For this example we are using tag36h11, ID 2 with a tag size of 30mm
% Undistort image <--- Undistort using lens parameters included in cameraParams
im = undistortImage(imDist,cameraParams);

% Parse Intrinsic Matrix
A_c2m = cameraParams.IntrinsicMatrix.'; % <-- Note the transpose

% Find tag36h11 AprilTag(s) in an image and estimate tag pose
tagFamily = 'tag36h11';
tagID = 2;
tagSize = 30.0;
[id,loc,pose,detectedFamily] = readAprilTag(im,tagFamily,cameraParams,tagSize);

% Plot image
fig = figure;
axs = axes('Parent',fig);
hold(axs,'on');
axis(axs,'tight');
img = imshow(im,'Parent',axs);

% Highlight and label AprilTags
for i = 1:numel(id)
    switch id(i)
    case tagID
    % Highlight AprilTag(s) matching tagID in blue
    ptc(i) = patch(axs,'Vertices',loc(:,:,i),'Faces',1:4,...
    'EdgeColor','m','FaceColor','b','FaceAlpha',0.1);

    % Define AprilTag label
    str = sprintf('%s\nID: %03d',detectedFamily(i),id(i));

    % Label AprilTag
    ang = rad2deg( atan2(...
    loc(2,1,i) - loc(1,1,i),...
    loc(2,2,i) - loc(1,2,i)) - pi/2 );
    txt = text(axs,mean(loc(:,1,i)),mean(loc(:,2,i)),str,...
        'HorizontalAlignment','center','VerticalAlignment','bottom',...
        'Rotation',ang,'FontWeight','Bold','FontSize',10,'Color','b');

    % Recover tag extrinsic matrix
    H_a2c = pose(i).T.';

    % Define projection matrix
    P_a2m = A_c2m * H_a2c(1:3,:);

    % Project x/y axes of Frame a into the image
    xy_a = 1.2*tagSize*[...
    0, 1, 0;...
    0, 0, 1;...
    0, 0, 0];
    xy_a(4,:) = 1;
    sxy_m = P_a2m*xy_a;
    xy_m = sxy_m./sxy_m(3,:);

    % Plot x-axis
    pltx = plot(axs,xy_m(1,[1,2]),xy_m(2,[1,2]),'r','LineWidth',1.5);
    % Plot y-axis
    plty = plot(axs,xy_m(1,[1,3]),xy_m(2,[1,3]),'g','LineWidth',1.5);
    otherwise
    % Highlight AprilTag(s) NOT matching tagID in red
    ptc(i) = patch(axs,'Vertices',loc(:,:,i),'Faces',1:4,...
        'EdgeColor','r','FaceColor','r','FaceAlpha',0.1);
    end
end

%% Find and recover alls poses of AprilTags

% Take image
imDist = get(prv,'CData'); % <-- Distorted image

% All available tag families with id and scale
tags = {'tag16h5' 0 10.0; 
        'tag16h5' 1 20.0;
        'tag16h5' 2 30.0;
        'tag16h5' 3 10.0;
        'tag25h9' 0 10.0;
        'tag25h9' 1 20.0;
        'tag25h9' 2 30.0;
        'tag25h9' 3 15.0;
        'tag36h11' 0 10.0;
        'tag36h11' 1 20.0;
        'tag36h11' 2 30.0;
        'tag36h11' 3 21.0;
        'tagCustom48h12' 0 40.0;
        'tagCustom48h12' 1 60.0;
        'tagCustom48h12' 2 80.0};

for i = 1:length(tags)
    % Parse through available tags
    tagFamily = tags{i, 1};
    tagID = tags{i, 2};
    tagSize = tags{i, 3};
    
    H_a2c = getAprilTagPose(imDist,cameraParams,tagFamily,tagID,tagSize); % find AprilTag Pose

    % Check if pose exists
    if isempty(H_a2c)
        continue
    end

    H_a2o = H_c2o * H_a2c; % calculate AprilTag pose relative to UR3e base
    X = poseToTask(H_a2o); % calculate task configuration to reach tag
    
    X_with_offset = X + [0; 0; 175; 0]; % task configuration with end-effector offset
    
    q = ikinPickAndPlace(X_with_offset,L); % calculate ikin
    
    ur.Joints = q; % move UR3e manipulator
    
end