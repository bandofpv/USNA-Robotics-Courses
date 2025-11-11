
function [board] = updateBoard(imDist,cameraParams, board)

% Board Grid (pose=elementToboard): gridRow, gridCol, X, Y
boardGrid = [1 1 -63.0 -63.0;
             1 2   0.0 -63.0;
             1 3  63.0 -63.0;
             2 1 -63.0   0.0;
             2 2   0.0   0.0;
             2 3  63.0   0.0;
             3 1 -63.0  63.0;
             3 2   0.0  63.0;
             3 3  63.0  63.0;];

end