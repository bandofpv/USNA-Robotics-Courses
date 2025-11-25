% Input 1: ur - URQt object of UR3e manipulator
% Input 2: canGrab - Nx5 cell array of Piece AprilTags not on the board 
% (Tag Family, Tag ID, Tag Scale, 'O'/'X', 4x4 pose relative to camera)
% Input 3: H_c2o - 4x4 array specifying the pose of the camera frame 
% relative to the Board frame (at board's center)
% Input 4: pieceType - character specifying piece type either 'X' or 'O'
% Input 5: gridLoc - 1x2 array specifying grid location to move piece on
% [gridRow, gridCol]

function movePiece(ur, canGrab, H_c2o, H_b2c, pieceType, gridLoc)
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

    % Find index of piece with matching pieceType
    pieceIdx = find(strcmp(canGrab(:, 4), pieceType));

    % Find index of grid location with matching gridLoc
    gridRow = gridLoc(1);
    gridCol = gridLoc(2);
    boardGridRows = boardGrid(:, 1) == gridRow;
    boardGridCols = boardGrid(:, 2) == gridCol;
    boardGridIdx = find(boardGridRows & boardGridCols);

    % Check if needed pieceType can be grabbed
    if pieceIdx
        % Calculate pose of piece (target) relative to base frame

        % Get pose of piece relative to camera
        H_t2c = canGrab(pieceIdx(end), 5); % get pose of piece
        H_t2o = H_c2o * cell2mat(H_t2c); % relative to UR3e base frame

        pickUpPiece(ur, H_t2o); % pick up piece

        % Calculate pose of grid location (target) relative to base frame

        % Point at board grid element relative to board's center
        p_t2b = [boardGrid(boardGridIdx, 3); boardGrid(boardGridIdx, 4); 0]; 

        % Point at board grid element relative to camera frame
        p_t2c = H_b2c * [p_t2b; 1]; 

        % Define pose of board grid element relative to camera frame
        H_t2c = H_b2c; % copy board pose (need rotation matrix)
        H_t2c(1:3,4) = p_t2c(1:3,:); % update position vector

        H_t2o = H_c2o * H_t2c; % relative to UR3e base frame

        placeDownPiece(ur, H_t2o); % place piece down

        ur.Home; % go back home
    else
        fprintf("'%c' piece cannot be grabbed (either not off board" + ...
            " or not in the camera frame)", pieceType);
    end
end

% Input 1: H_t2o - 4x4 array specifying the pose of the piece (target) 
% relative to the Board frame (at board's center)

function pickUpPiece(ur, H_t2o)
        L = [151.85, 243.55, 213.2, 131.05, 85.35, 92.10]; % UR3e link lengths
        tableHeight = 107.0669;
        aboveOffset = 30; % offset to go right above target
        gp = 13.4588235294118; % gripper position to grab piece

        % Move UR3e right over piece
        X = poseToTask(H_t2o); % calculate task configuration to reach tag
        X(3) = tableHeight + aboveOffset; % update z with offset
        q = ikinPickAndPlace(X,L); % calculate ikin with offset  
        ur.Joints = q; % move UR3e manipulator

        % Move UR3e to grab piece
        X = poseToTask(H_t2o); % calculate task configuration to reach tag
        X(3) = tableHeight; % update z 
        q = ikinPickAndPlace(X,L); % calculate ikin with offset  
        ur.Joints = q; % move UR3e manipulator

        % Grab piece
        ur.GripPosition = gp; % close gripper

        % Move UR3e right over piece
        X = poseToTask(H_t2o); % calculate task configuration to reach tag
        X(3) = tableHeight + aboveOffset; % update z with offset
        q = ikinPickAndPlace(X,L); % calculate ikin with offset  
        ur.Joints = q; % move UR3e manipulator
end

% Input 1: H_t2o - 4x4 array specifying the pose of the grid location 
% (target) relative to the Board frame (at board's center)

function placeDownPiece(ur, H_t2o)
        L = [151.85, 243.55, 213.2, 131.05, 85.35, 92.10]; % UR3e link lengths
        boardHeight = 115.2337;
        aboveOffset = 30; % offset to go right above target

        % Move UR3e right over grid location
        X = poseToTask(H_t2o); % calculate task configuration to reach tag
        X(3) = boardHeight + aboveOffset; % update z with offset  
        q = ikinPickAndPlace(X,L); % calculate ikin with offset  
        ur.Joints = q; % move UR3e manipulator

        % Move UR3e to drop piece
        X = poseToTask(H_t2o); % calculate task configuration to reach tag
        X(3) = boardHeight; % update z  
        q = ikinPickAndPlace(X,L); % calculate ikin with offset  
        ur.Joints = q; % move UR3e manipulator

        % Drop piece
        ur.GripPosition = 0; % open gripper

        % Move UR3e right over grid location
        X = poseToTask(H_t2o); % calculate task configuration to reach tag
        X(3) = boardHeight + aboveOffset; % update z with offset  
        q = ikinPickAndPlace(X,L); % calculate ikin with offset  
        ur.Joints = q; % move UR3e manipulator
end