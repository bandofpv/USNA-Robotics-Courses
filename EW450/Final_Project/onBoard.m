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