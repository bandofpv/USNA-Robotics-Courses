% Input 1: pieceTags - Nx4 cell array specifying Piece AprilTags (Tag 
% Family, Tag ID, Tag Scale, O/X)
% Output 1: tags - Nx4 cell array specifying Piece AprilTags (Tag 
% Family, Tag ID, Tag Scale, O/X) available for grabbing

function [tags] = canGrab(pieceTags, H_b2o)
    boardDim = 94.5; % x & y distance from center of board to edge
    
    % Parse pose for x and y translations
    t_x = H_p2b(1, 4);
    t_y = H_p2b(2, 4);

    % Check if the piece is on the board
    if t_x > -boardDim && t_x < boardDim && t_y > -boardDim && t_y < boardDim
        onBoard = True;
    else
        onBoard = False;
    end
end

%% THIS IS A WIP