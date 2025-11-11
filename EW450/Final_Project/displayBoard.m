% Input 1: board - 3x3 array of strings either empty, 'X', or 'O'

function displayBoard(board)
    fprintf('\n');
    for i = 1:3
        fprintf(' %c | %c | %c ', board(i, 1), board(i, 2), board(i, 3));
        if i < 3
            fprintf('\n---|---|---\n');
        end
    end
    fprintf('\n\n');
end