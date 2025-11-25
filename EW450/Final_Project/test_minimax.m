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