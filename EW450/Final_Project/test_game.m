% Main function to run the Tic-Tac-Toe game
board = repmat(' ', 3, 3); % Initialize a 3x3 empty board
human = 'X';
ai = 'O';

fprintf('Welcome to Tic-Tac-Toe!\n');
fprintf('You are %c. The AI is %c.\n', human, ai);

% Randomly decide who goes first
if randi(2) == 1
    currentPlayer = human;
    fprintf('You go first.\n');
else
    currentPlayer = ai;
    fprintf('The AI goes first.\n');
end

displayBoard(board);

while true
    if currentPlayer == human
        % Human's turn
        move = input('Enter your move (row and col, e.g., [1 2]): ');
        row = move(1);
        col = move(2);
        
        if row < 1 || row > 3 || col < 1 || col > 3 || board(row, col) ~= ' '
            fprintf('Invalid move. Try again.\n');
            continue;
        end
        board(row, col) = human;
    else
        % AI's turn
        fprintf('AI is thinking...\n');
        [bestMoveRow, bestMoveCol] = findBestMove(board, ai, human);
        board(bestMoveRow, bestMoveCol) = ai;
        fprintf('AI plays at (%d, %d)\n', bestMoveRow, bestMoveCol);
    end

    displayBoard(board);

    % Check for game over condition
    winner = checkWinner(board);
    if winner ~= ' '
        if winner == human
            fprintf('Congratulations! You won!\n');
        else
            fprintf('The AI wins! Better luck next time.\n');
        end
        break;
    elseif ~isMovesLeft(board)
        fprintf('It''s a draw!\n');
        break;
    end

    % Switch turns
    if currentPlayer == human
        currentPlayer = ai;
    else
        currentPlayer = human;
    end
end

% --- Helper Functions ---

% Checks if there are any moves left on the board
function movesLeft = isMovesLeft(board)
    movesLeft = any(board(:) == ' ');
end

% The minimax function
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

% Finds the best move for the robot using the minimax algorithm
function [bestRow, bestCol] = findBestMove(board, robotPlayer, humanPlayer)
    bestVal = -inf;
    bestRow = -1;
    bestCol = -1;

    for i = 1:3
        for j = 1:3
            if board(i, j) == ' ' % check if cell is empty
                board(i, j) = robotPlayer; % make the move

                % Compute minimax score for this move
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
