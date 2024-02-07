function interpolated_matrix = interpolate_bilinear(matrix)
    mask = ones(3); % Define a 3x3 mask of ones
    
    % Replace zeros with NaNs to ignore them during convolution
    matrix_with_nan = double(matrix);
    matrix_with_nan(matrix_with_nan == 0) = NaN;
    
    % Perform convolution with the mask to get the sum of non-zero neighbors
    sum_nonzero_neighbors = conv2(matrix, mask, 'same');
    
    % Count the number of non-zero neighbors (excluding the central pixel)
    count_nonzero_neighbors = conv2(~isnan(matrix_with_nan), mask, 'same');
    
    % Calculate the mean of non-zero neighbors
    mean_nonzero_neighbors = sum_nonzero_neighbors ./ count_nonzero_neighbors;
    
    % Replace zeros in the original matrix with the mean of non-zero neighbors
    interpolated_matrix = matrix;
    interpolated_matrix(interpolated_matrix == 0) = mean_nonzero_neighbors(interpolated_matrix == 0);
end