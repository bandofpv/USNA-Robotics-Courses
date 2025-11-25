% Input 1: board - 3x3 array of characters either ' ', 'X', or 'O'
% Output 1: movesLeft - true/false if there are any moves left to play on 
% the board

function movesLeft = isMovesLeft(board)
    movesLeft = any(board(:) == ' '); % check for any empty grid cells
end