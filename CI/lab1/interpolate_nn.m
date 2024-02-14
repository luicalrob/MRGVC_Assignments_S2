function interpolated_matrix = interpolate_nn(matrix, color)
    % Create masks for zero elements
    zero_mask = matrix == 0;
    
    % Perform interpolation for each color channel
    switch color
        case 'green'
            % INTENTAR PREGUNTAR O LO QUE SEA
            % Interpolate using the average of non-zero elements in the 2x2 neighborhood
            green_avg = (circshift(matrix, [0, -1]) + circshift(matrix, [-1, 0]) + circshift(matrix, [1, 0]) + circshift(matrix, [0, 1])) / 4;
            interpolated_matrix = matrix;
            interpolated_matrix(zero_mask) = green_avg(zero_mask);
        case 'red'
            % Interpolate using the value from the top-left non-zero element
            interpolated_matrix = circshift(matrix, [1, 1])+circshift(matrix, [1, 0])+circshift(matrix, [0, 1])+matrix;
        case 'blue'
            % Interpolate using the value from the bottom-right non-zero element
            interpolated_matrix = circshift(matrix, [-1, -1])+circshift(matrix, [-1, 0])+circshift(matrix, [0, -1])+matrix;
        otherwise
            error('Invalid color specified.');
    end
end
