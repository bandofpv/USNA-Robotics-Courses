% Input 1: board - 3x3 array of strings either ' ', 'X', or 'O'
% Input 2: robotPlayer - String representing robot's peices either ' ', 
%   'X', or 'O'
% Input 3: humanPlayer - String representing humans's peices either ' ', 
%   'X', or 'O'
% Output 1: bestRow - Integer representing row index of best move
% Output 1: bestCol - Integer representing colum index of best move

function [bestRow, bestCol] = findBestMove(board, robotPlayer, humanPlayer)
    bestVal = -inf;
    bestRow = -1;
    bestCol = -1;

    for i = 1:3
        for j = 1:3
            if board(i, j) == ' ' % check if cell is empty
                board(i, j) = robotPlayer; % make the move

                % Compute minimax value for this move
                moveVal = minimax(board, 0, false, robotPlayer, humanPlayer);

                board(i, j) = ' '; % undo move
                
                % update best move if current move is best
                if moveVal > bestVal
                    bestRow = i;
                    bestCol = j;
                    bestVal = moveVal;
                end
            end
        end
    end
end

% Input 1: board - 3x3 array of strings either ' ', 'X', or 'O'
% Output 1: movesLeft - True/False if there are any moves left to play

function movesLeft = isMovesLeft(board)
    movesLeft = any(board(:) == ' ');
end

% Input 1: board - 3x3 array of strings either ' ', 'X', or 'O'
% Input 2: depth - Integer representing number of moves the algorithm looks 
%   ahead in the game tree
% Input 3: isMax - True/False if maximizer's (robot) move
% Input 4: robotPlayer - String representing robot's peices either ' ', 
%   'X', or 'O'
% Input 5: humanPlayer - String representing humans's peices either ' ', 
%   'X', or 'O'
% Output 1: score - Integer representing value of move

function score = minimax(board, depth, isMax, robotPlayer, humanPlayer)
    winner = checkWinner(board);

    % If maximizer (robot) has won the game, return their score
    if winner == robotPlayer
        score = 10 - depth;
        return;
    % If minimizer (human) has won the game, return their score
    elseif winner == humanPlayer
        score = depth - 10;
        return;
    % If there are not more move and no winner, tie
    elseif ~isMovesLeft(board)
        score = 0;
        return;
    end

    % Maximizer's move (robot)
    if isMax
        best = -inf;
        for i = 1:3
            for j = 1:3
                if board(i, j) == ' ' % check if cell is empty
                    board(i, j) = robotPlayer; % make the move
                    
                    % Call minimax recusively and choose the maximum value
                    best = max(best, minimax(board, depth + 1, ~isMax, robotPlayer, humanPlayer));

                    board(i, j) = ' '; % undo move
                end
            end
        end
        score = best;

    % Minimizer's move (Human)
    else
        best = inf;
        for i = 1:3
            for j = 1:3
                if board(i, j) == ' ' % check if cell is empty
                    board(i, j) = humanPlayer; % make the move

                    % Call minimax recusively and choose the maximum value
                    best = min(best, minimax(board, depth + 1, ~isMax, robotPlayer, humanPlayer));

                    board(i, j) = ' '; % undo move
                end
            end
        end
        score = best;
    end
end