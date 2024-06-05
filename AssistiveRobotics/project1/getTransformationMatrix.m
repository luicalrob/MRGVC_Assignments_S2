function pose = getTransformationMatrix(data)
    % Extract the position coordinates and Euler angles from the input vector
    x = data(1);
    y = data(2);
    z = data(3);
    roll = data(4);
    pitch = data(5);
    yaw = data(6);

    % Convert Euler angles to a rotation matrix manually
    % Define the individual rotation matrices
    Rz = [cos(yaw), -sin(yaw), 0;
          sin(yaw),  cos(yaw), 0;
          0,        0,        1];
    Ry = [cos(pitch), 0, sin(pitch);
          0,          1, 0;
         -sin(pitch), 0, cos(pitch)];
    Rx = [1, 0,        0;
          0, cos(roll), -sin(roll);
          0, sin(roll),  cos(roll)];

    % Combine the rotations in the order ZYX
    R = Rz * Ry * Rx;

    % Combine rotation matrix and translation vector into a homogeneous transformation matrix
    T = [R, [x; y; z]; 0 0 0 1];

    % Create the SE3 object from the homogeneous transformation matrix
    pose = SE3(T);
end
