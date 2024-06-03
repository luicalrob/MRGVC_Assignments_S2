function cartesian_curve_T = getCurveInterpolation(vectorName)
    points = getData(vectorName);
    
    % Known orientation indices and their corresponding values in radians
    known_indices = [1, 11, 18];  % MATLAB uses 1-based indexing
    known_roll = [-2.7236, -2.7796, -2.8423];
    known_pitch = [-0.4138, -0.2365, -0.5434];
    known_yaw = [-1.0834, -0.7604, 0.1827];
    
    % Vector of all indices for interpolation
    full_indices = 1:size(points, 1);
    
    % Interpolate orientations using linear interpolation
    interpolated_roll = interp1(known_indices, known_roll, full_indices, 'linear', 'extrap');
    interpolated_pitch = interp1(known_indices, known_pitch, full_indices, 'linear', 'extrap');
    interpolated_yaw = interp1(known_indices, known_yaw, full_indices, 'linear', 'extrap');
    
    % Combine points and orientations into one matrix
    cartesian_curve = [points, interpolated_roll', interpolated_pitch', interpolated_yaw'];
    cartesian_curve_T = repmat(SE3(), 1, size(cartesian_curve, 1));
    % Display the combined data
    for i = 1:size(cartesian_curve, 1)
        cartesian_curve_T(i) = getTransformationMatrix(cartesian_curve(i, :));
    end

end