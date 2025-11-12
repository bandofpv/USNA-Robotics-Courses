% Input 1: imDist - image captured directly from camera.
% Input 2: cameraParams - MATLAB camera parameters object.
% Output 1: H_b2c - 4x4 array specifying the pose of the Board frame (at
% board's center) relative to the camera frame.

function [H_b2c] = getBoardPose(imDist,cameraParams)
    % Board tags (pose=centerTotag): Tag Family, Tag ID, Tag Scale, X, Y
    tags = {'tag36h11' 450 29.0  75.0 -115.0; 
            'tag36h11' 460 29.0 -75.0 -115.0};
    centerPoses = cell(2,1); % empty 2x1
    missingTag = false;
    H_b2c = [];

    % Parse Intrinsic Matrix
    A_c2m = cameraParams.IntrinsicMatrix.'; % <-- Note the transpose

    % Undistort image <--- Undistort using lens parameters included in cameraParams
    im = undistortImage(imDist,cameraParams);

    % Parse through available tags
    for i = 1:size(tags, 1)
        tagFamily = tags{i, 1};
        tagID = tags{i, 2};
        tagSize = tags{i, 3};

        % Point at board's center relative to AprilTag
        p_i2a = [tags{i, 4}; tags{i, 5}; 0]; 
        
        % find AprilTag pose
        H_a2c = getAprilTagPose(im,cameraParams,tagFamily,tagID,tagSize); 

        % Define point relative to camera frame
        p_i2c = H_a2c * [p_i2a; 1]; 

        % Define center pose relative to camera frame
        H_i2c = H_a2c; % copy AprilTag pose
        H_i2c(1:3,4) = p_i2c(1:3,:); % define position of board center

        % Check if pose exists
        if isempty(H_a2c)
            missingTag = True;
        else
            H_b2c = H_i2c; % save pose in case a tag is missing
            centerPoses{i} = H_i2c; % add to cell to average later
        end
    end

    % Average pose if both tags exist
    if ~missingTag
        % Parse homogenous matricies
        t_1 = centerPoses{1}(1:3, 4);
        t_2 = centerPoses{2}(1:3, 4);
       
        t_avg = (t_1 + t_2) / 2; % average translation

        H_b2c(1:3, 4) = t_avg;
    end

    % Plot image
    fig = figure;
    axs = axes('Parent',fig);
    hold(axs,'on');
    axis(axs,'tight');
    img = imshow(im,'Parent',axs);

    % Define projection matrix
    P_b2m = A_c2m * H_b2c(1:3,:);

    % Project x/y axes of Board Frame into the image
    xy_b = 1.2*tagSize*[...
    0, 1, 0;...
    0, 0, 1;...
    0, 0, 0];
    xy_b(4,:) = 1;
    sxy_m = P_b2m*xy_b;
    xy_m = sxy_m./sxy_m(3,:);

    % Plot x-axis
    pltx = plot(axs,xy_m(1,[1,2]),xy_m(2,[1,2]),'r','LineWidth',2);
    % Plot y-axis
    plty = plot(axs,xy_m(1,[1,3]),xy_m(2,[1,3]),'g','LineWidth',2);
end