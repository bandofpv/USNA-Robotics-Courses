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

ur.Home; % go to home
ur.GripPosition = 0; % open gripper

%% Calibrate Camera and Robot

[H_o2c,H_c2o] = calibrateCameraAndRobot(prv,ur,cameraParams,nImages);

%% Save Calibration

a = datetime;
fname = sprintf('RobotCameraCal_%s.mat',string(a,'ddMMMyyyy_hhmmss'));
save(fname,'cameraParams','H_o2c','H_c2o');

%% Capture Image

imDist = get(prv,'CData'); % <-- Distorted image
% imDist = imbinarize(rgb2gray(imDist), 'adaptive', 'ForegroundPolarity', 'dark', 'Sensitivity', 0.4);

%% Initizlize Camera

[cam,prv,handles] = initCamera;
camSettings = adjustCamera(cam);
load("RobotCameraCal_18Nov2025_085059.mat")

%% Initialize UR3e

ur = URQt('UR3e');
ur.Initialize;

ur.Home; % go to home
ur.GripPosition = 0; % open gripper
ur.GripPosition = 52; % close gripper
ur.GripPosition = 0; % open gripper

%% Main Script

% Initialize arrays
board = repmat(' ', 3, 3); % empty board
inFOV = {};

fprintf("Welcome to Tic-Tac-Toe!");

[board, inFOV, canGrab, H_b2c] = updateBoard(prv, cameraParams, board, inFOV);

% Prompt player to select piece
while true
    humanPieceType = upper(input("\nPlease enter what piece you would " + ...
        "like to play with (X or O): ", 's'));
    
    if humanPieceType == 'X'
        fprintf("Great! You chose the Red X pieces!\n");
        robotPieceType = 'O';
        break
    elseif humanPieceType == 'O'
        fprintf("Great! You chose the Blue O pieces!\n");
        robotPieceType = 'X';
        break
    else
        fprintf("Enter the character 'X' or 'O'.");
    end
end

while true
    robotStart = input("\nWould you like to robot to start " + ...
        "first? Type yes or no: ", 's');

    if strcmpi(robotStart, "yes")
        % fprintf("Robot playing first move...\n");
        % 
        % % Play a random move to start game
        % randomRow = randi([1 3]);
        % randomCol = randi([1 3]);
        % movePiece(ur, canGrab, H_c2o, H_b2c, robotPieceType, [randomRow, randomCol]);
        % fprintf('Robot plays at (%d, %d)\n', randomRow, randomCol);
        % 
        % % Update board
        % [board, inFOV, canGrab, H_b2c] = updateBoard(prv, cameraParams, board, inFOV);
        % displayBoard(board);
        % 
        % currentPlayer = humanPieceType; % switch turns
        currentPlayer = robotPieceType;
        break
    elseif strcmpi(robotStart, "no")
        currentPlayer = humanPieceType;
        break
    else
        fprintf("Enter 'yes' or 'no'.");
    end

end

while true
    % Robot's turn to play
    if currentPlayer == robotPieceType
        % Calculate best move to play
        fprintf('Robot is thinking...\n');
        [bestMoveRow, bestMoveCol] = findBestMove(board, robotPieceType, humanPieceType);

        % Play the best move
        movePiece(ur, canGrab, H_c2o, H_b2c, robotPieceType, [bestMoveRow, bestMoveCol]);
        fprintf('Robot plays at (%d, %d)\n', bestMoveRow, bestMoveCol);

        currentPlayer = humanPieceType; % switch turns
    % Human's turn to play
    elseif currentPlayer == humanPieceType
        fprintf("Please place a '%c' piece on the board\n", humanPieceType);

        % Wait for human to play
        while true
            originalBoard = board;
            [board, inFOV, ~, ~] = updateBoard(prv, cameraParams, originalBoard, inFOV);

            % Check if the player placed a new piece on the board
            if ~isequal(originalBoard, board)
                break
            end

            pause(1); % pause for one second
        end

        currentPlayer = robotPieceType; % switch turns
    end
    
    [board, inFOV, canGrab, H_b2c] = updateBoard(prv, cameraParams, board, inFOV);
    displayBoard(board);

    winner = checkWinner(board);
    % Check if there is a winner
    if winner ~= ' '
        if winner == robotPieceType
            fprintf('The Robot wins! Better luck next time.\n');    
            break
        elseif winner == humanPieceType
            fprintf('Congratulations! You won!\n');
            break
        end
    % Check if no more moves left
    elseif ~isMovesLeft(board)
        fprintf("It's a draw!\n");
        break
    end
end

fprintf("Thank you for playing Tic-Tac-Toe!\n")

%% TURN CHECK LOGIC CHECK INDIVIDUAL PIECE RATHER THAN OVERALL CHANGE
