

function movePiece(ur, canGrab, H_c2o, pieceType, gridLoc)
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

    % Check if needed pieceType can be grabbed
    if pieceIdx
        % Calculate pose of piece (target) relative to base frame
        H_t2c = canGrab(pieceIdx(end), 5);
        H_t2o = H_c2o * H_t2c;

        pickUpPiece(ur, H_t2o); % pick up piece

        % Calculate pose of grid location (target) relative to base frame
        H_t2o = 0;

        % NOTE: Code for grid to board pose is in updateBoard.m

        placeDownPiece(ur, H_t2o); % place piece down

        ur.Home; % go back home
    else
        sprintf("'%c' piece cannot be grabbed (either not off board" + ...
            " or not in the camera frame)", pieceType)
    end
end

function pickUpPiece(ur, H_t2o)
        % Move UR3e right over piece

        % Move UR3e to grab piece

        % Grab piece

        % Move UR3e right over piece

end

function placeDownPiece(ur, H_t2o)
        % Move UR3e right over grid location

        % Move UR3e to drop piece

        % Drop piece

        % Move UR3e right over grid location

end