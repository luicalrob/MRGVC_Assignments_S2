function data = getData(vectorName)

% inicial 1
% 8
% reposo 1
% 10
% arriba 1
% 4
% tenso1 1
% 4
% tenso 1
% 6
% final 1

    % Define the data vectors inside the function

    %% LEFT
    known_indices = [1, 10, 21, 31, 38];
    points = [
        0.25, 0.37, -0.2; % A
        0.32, 0.38, -0.17;
        0.38, 0.37, -0.14;
        0.44, 0.35, -0.12;
        0.52, 0.32, -0.08;
        0.58, 0.26, -0.05;
        0.63, 0.21, -0.03;
        0.67, 0.14, 0.0;
        0.69, 0.05, 0.02;
        0.7, -0.03, 0.03; % B 
        0.7, -0.08, 0.05;
        0.69, -0.12, 0.07;
        0.68, -0.17, 0.09;
        0.67, -0.21, 0.12;
        0.65, -0.27, 0.16;
        0.63, -0.32, 0.2;
        0.61, -0.39, 0.27;
        0.57, -0.44, 0.35;
        0.54, -0.49, 0.43;
        0.5, -0.52, 0.53;
        0.45, -0.54, 0.63; % C
        0.55, -0.47, 0.47;
        0.59, -0.37, 0.33;
        0.63, -0.27, 0.20;
        0.65, -0.21, 0.14;
        0.66, -0.14, 0.08;
        0.67, -0.07, 0.04;
        0.67, -0.03, 0.02;
        0.67, 0.01, 0.01;
        0.67, 0.04, 0.00;
        0.67, 0.08, 0.00; % D
        0.65, 0.15, 0.00;
        0.64, 0.21, 0.02;
        0.61, 0.28, 0.06;
        0.58, 0.35, 0.11;
        0.54, 0.42, 0.18;
        0.50, 0.48, 0.27;
        0.40, 0.61, 0.45; % E
    ];
    
    % known_roll = [3.1416, 2.9023, 2.7332, -2.7796, -2.8423];
    known_roll = [-3.13, -3.13, -3.13, -2.7796, -2.8423];
    known_pitch = [-0.00001, -0.0149, -1.0975, -0.2365, -0.5434];
    known_yaw = [-0.00001, -0.1050, -0.7019, -0.7604, 0.1827];
    
    % Vector of all indices for interpolation
    full_indices = 1:size(points, 1);
    
    % Interpolate orientations using linear interpolation
    interpolated_roll = interp1(known_indices, known_roll, full_indices, 'linear', 'extrap');
    interpolated_pitch = interp1(known_indices, known_pitch, full_indices, 'linear', 'extrap');
    interpolated_yaw = interp1(known_indices, known_yaw, full_indices, 'linear', 'extrap');
    
    % Combine points and orientations into one matrix
    left_cartesian_curve = [points, interpolated_roll', interpolated_pitch', interpolated_yaw'];
    
    %% RIGHT
    points = [
        0.25, -0.37, -0.2; % A
        0.35, -0.34, -0.15;
        0.43, -0.3, -0.11;
        0.51, -0.24, -0.07;
        0.56, -0.2, -0.05;
        0.59, -0.16, -0.03;
        0.63, -0.11, -0.01;
        0.66, -0.05, 0.01;
        0.69, -0.01, 0.02;
        0.7, 0.03, -0.03; % B
        0.7, 0.0, 0.04;
        0.68, -0.05, 0.07;
        0.67, -0.1, 0.09;
        0.65, -0.15, 0.12;
        0.63, -0.2, 0.16;
        0.62, -0.25, 0.2;
        0.6, -0.3, 0.24;
        0.58, -0.34, 0.28;
        0.56, -0.38, 0.34;
        0.52, -0.45, 0.44;
        0.45, -0.54, 0.63; % C
        0.55, -0.47, 0.47;
        0.59, -0.37, 0.33;
        0.63, -0.27, 0.20;
        0.65, -0.21, 0.14;
        0.66, -0.14, 0.08;
        0.67, -0.07, 0.04;
        0.67, -0.03, 0.02;
        0.67, 0.01, 0.01;
        0.67, 0.04, 0.00;
        0.67, 0.08, 0.00; % D
        0.65, 0.15, 0.00;
        0.64, 0.21, 0.02;
        0.61, 0.28, 0.06;
        0.58, 0.35, 0.11;
        0.54, 0.42, 0.18;
        0.50, 0.48, 0.27;
        0.40, 0.61, 0.45; % E
    ];
    
    % known_roll = [3.1416, -2.9023, 3.0626, 3.1000, -3.0269, -2.7527];
    known_roll = [-3.13, -2.9023, -3.13, -3.0269, -2.7527];
    known_pitch = [-0.00001, -0.0149, -1.1881, -0.0986, -0.5563];
    % known_yaw = [0.00001, 0.1050, -0.3368, 0.6028, 0.5904, 0.7334];
    known_yaw = [0.00001, 0.1050, 0.00001, 0.5904, 0.7334];
    
    % Vector of all indices for interpolation
    full_indices = 1:size(points, 1);
    
    % Interpolate orientations using linear interpolation
    interpolated_roll = interp1(known_indices, known_roll, full_indices, 'linear', 'extrap');
    interpolated_pitch = interp1(known_indices, known_pitch, full_indices, 'linear', 'extrap');
    interpolated_yaw = interp1(known_indices, known_yaw, full_indices, 'linear', 'extrap');
    
    % Combine points and orientations into one matrix
    right_cartesian_curve = [points, interpolated_roll', interpolated_pitch', interpolated_yaw'];
    
    
    
    % Return the requested vector based on input argument
    switch vectorName
        case 'left'
            cartesian_curve_T = repmat(SE3(), 1, size(left_cartesian_curve, 1));
            % Display the combined data
            for i = 1:size(left_cartesian_curve, 1)
                cartesian_curve_T(i) = getTransformationMatrix(left_cartesian_curve(i, :));
            end
        case 'right'
            cartesian_curve_T = repmat(SE3(), 1, size(right_cartesian_curve, 1));
            % Display the combined data
            for i = 1:size(right_cartesian_curve, 1)
                cartesian_curve_T(i) = getTransformationMatrix(right_cartesian_curve(i, :));
            end
        otherwise
            error('Unknown data requested.');
    end
    
    data = cartesian_curve_T;
end
