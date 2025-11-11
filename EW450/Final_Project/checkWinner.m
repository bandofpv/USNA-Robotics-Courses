% Input 1: board - 3x3 array of strings either ' ', 'X', or 'O'
% Output 1: winner - tic tac toe winner: ' ', 'X', or 'O'

function winner = checkWinner(board)
    winner = ' '; % initialize winner

    % Check rows
    for i = 1:3
        if all(board(i, :) == board(i, 1)) && board(i, 1) ~= ' '
            winner = board(i, 1); 
            return;
        end
    end
    
    % Check cols
    for i = 1:3
        if all(board(:, i) == board(1, i)) && board(1, i) ~= ' '
            winner = board(1, i); 
            return;
        end
    end

    % Check diagonal
    if all(diag(board) == board(1, 1)) && board(1, 1) ~= ' '
        winner = board(1, 1); 
        return;
    end

    % Check anti-diagonal
    if all(diag(fliplr(board)) == board(1, 3)) && board(1, 3) ~= ' '
        winner = board(1, 3); 
        return;
    end
end