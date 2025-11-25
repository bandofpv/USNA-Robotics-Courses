% Input 1: prv - preview object from camera
% Input 2: cameraParams - MATLAB camera parameters object.
% Input 3: board - 3x3 array of characters either ' ', 'X', or 'O'
% Output 1: board - 3x3 array of characters either ' ', 'X', or 'O'
% Output 2: inFOV - Nx4 cell array of Piece AprilTags in the camera FOV
% (Tag Family, Tag ID, Tag Scale, 'O'/'X')
% Output 3: canGrab - Nx5 cell array of Piece AprilTags not on the board 
% (Tag Family, Tag ID, Tag Scale, 'O'/'X', 4x4 pose relative to camera)
% Output 4: H_b2c - 4x4 array specifying the pose of the Board frame (at
% board's center) relative to the camera frame.

function [board, inFOV, canGrab, H_b2c] = updateBoard(prv, cameraParams, board, inFOV)

    boardTagSize = 29.0; % tag size of board AprilTag

    % Piece AprilTags:Tag Family, Tag ID, Tag Scale, 'O'/'X'
    pieces = {'tag36h11' 451 25.0 'O';
            'tag36h11' 452 25.0 'O';
            'tag36h11' 453 25.0 'O';
            'tag36h11' 454 25.0 'O';
            'tag36h11' 455 25.0 'O';
            'tag36h11' 461 25.0 'X';
            'tag36h11' 462 25.0 'X';
            'tag36h11' 463 25.0 'X';
            'tag36h11' 464 25.0 'X';
            'tag36h11' 465 25.0 'X'};
    
    % Board Grid (pose=elementToboard): gridRow, gridCol, X, Y
    boardGrid = [1 1 -63.0 -63.0;
                 1 2   0.0 -63.0;
                 1 3  63.0 -63.0;
                 2 1 -63.0   0.0;
                 2 2   0.0   0.0;
                 2 3  63.0   0.0;
                 3 1 -63.0  63.0;
                 3 2   0.0  63.0;
                 3 3  63.0  63.0];

    canGrab = {};
    
    % Parse Intrinsic Matrix
    A_c2m = cameraParams.IntrinsicMatrix.'; % <-- Note the transpose

    % Take image directly from camera
    imDist = get(prv,'CData'); % <-- Distorted image
    % imDist = imbinarize(rgb2gray(imDist), 'adaptive', 'ForegroundPolarity', 'dark', 'Sensitivity', 0.2);

    % Undistort image <--- Undistort using lens parameters included in cameraParams
    im = undistortImage(imDist,cameraParams);

    % Get pose of board's center
    H_b2c = getBoardPose(imDist,cameraParams);

    % Return if board is not detected
    if isempty(H_b2c)
        disp("Board pose not found!")
        return
    end

    % % COMMENT OUT AFTER (PLOTS BOARD CENTER POSE)
    % % Plot image
    % fig = figure;
    % axs = axes('Parent',fig);
    % hold(axs,'on');
    % axis(axs,'tight');
    % img = imshow(im,'Parent',axs);
    % 
    % % Define projection matrix
    % P_b2m = A_c2m * H_b2c(1:3,:);
    % 
    % % Project x/y axes of Board Frame into the image
    % xy_b = 1.2*boardTagSize*[...
    % 0, 1, 0;...
    % 0, 0, 1;...
    % 0, 0, 0];
    % xy_b(4,:) = 1;
    % sxy_m = P_b2m*xy_b;
    % xy_m = sxy_m./sxy_m(3,:);
    % 
    % % Plot x-axis
    % pltx = plot(axs,xy_m(1,[1,2]),xy_m(2,[1,2]),'r','LineWidth',2);
    % % Plot y-axis
    % plty = plot(axs,xy_m(1,[1,3]),xy_m(2,[1,3]),'g','LineWidth',2);

    % % COMMENT OUT AFTER (PLOTS GRID ELEMENT POSES)
    % for i = 1:length(boardGrid)
    % 
    %     % Point at board grid element relative to board's center
    %     p_i2b = [boardGrid(i, 3); boardGrid(i, 4); 0]; 
    % 
    %     % Point at board grid element relative to camera frame
    %     p_i2c = H_b2c * [p_i2b; 1]; 
    % 
    %     % Define pose relative to camera frame
    %     H_i2c = H_b2c; % copy board pose
    %     H_i2c(1:3,4) = p_i2c(1:3,:); % define position of board center
    % 
    %     % Define projection matrix
    %     P_i2m = A_c2m * H_i2c(1:3,:);
    % 
    %     % Project x/y axes of Board Frame into the image
    %     xy_i = 1.2*boardTagSize*[...
    %     0, 1, 0;...
    %     0, 0, 1;...
    %     0, 0, 0];
    %     xy_i(4,:) = 1;
    %     sxy_m = P_i2m*xy_i;
    %     xy_m = sxy_m./sxy_m(3,:);
    % 
    %     % Plot x-axis
    %     pltx = plot(axs,xy_m(1,[1,2]),xy_m(2,[1,2]),'r','LineWidth',2);
    %     % Plot y-axis
    %     plty = plot(axs,xy_m(1,[1,3]),xy_m(2,[1,3]),'g','LineWidth',2);
    % end

    % Parse through available tags
    for i = 1:size(pieces, 1)
        tagFamily = pieces{i, 1};
        tagID = pieces{i, 2};
        tagSize = pieces{i, 3};
        
        % find AprilTag pose
        H_a2c = getAprilTagPose(im,cameraParams,tagFamily,tagID,tagSize); 

        % Check if pose exists (tag recognized)
        if ~isempty(H_a2c)
            % Check if piece is on the board
            if onBoard(H_a2c, H_b2c)
                p_a2c = H_a2c(1:3,4); % point of tag relative to camera frame
    
                % Calcualte point of tag relative to board frame
                p_a2b = invSE(H_b2c) * [p_a2c; 1];
    
                % Calculate euclidean distance from tag to board grid element
                dist = pdist2(p_a2b(1:2)', boardGrid(:, 3:4), 'euclidean');
    
                [~, minIdx] = min(dist); % get min index
                boardRow = boardGrid(minIdx, 1); % get row of board  grid element
                boardCol = boardGrid(minIdx, 2); % get col of board  grid element
    
                board(boardRow, boardCol) = pieces{i, 4}; % update board grid element
                % displayBoard(board); 
            else
                % Add piece to canGrab if not on the board
                canGrab(end+1, 1:size(pieces, 2)) = pieces(i, :);
                canGrab{end, size(pieces, 2) + 1} = H_a2c; % add pose of piece
            end
            % If tag is recognized but not recorded in inFOV
            if isempty(find([inFOV{:}]==tagID, 1))
                inFOV(end+1, :) = pieces(i, :);
            end
        else
            % If tag is in inFOV but not recognized again
            if find([inFOV{:}]==tagID)
                fprintf("\nFailed to find '%c' piece ID: %d\n", pieces{i, 4}, tagID);
            end       
        end
    end    
end

% Input 1: H_a2c - 4x4 array specifying the pose of the piece relative to 
% the camera frame.
% Input 1: H_b2c - 4x4 array specifying the pose of the Board frame (at
% board's center) relative to the camera frame.
% Output 1: on - true/false specificy if piece is on the board

function [on] = onBoard(H_a2c, H_b2c)
    boardDim = 94.5; % x & y distance from center of board to edge

    % Calculate pose of tag relative to board frame
    H_a2b = invSE(H_b2c) * H_a2c;
    
    % Parse pose for x and y translations
    t_x = H_a2b(1, 4);
    t_y = H_a2b(2, 4);

    % Check if the piece is on the board
    if t_x > -boardDim && t_x < boardDim && t_y > -boardDim && t_y < boardDim
        on = true;
    else
        on = false;
    end
end