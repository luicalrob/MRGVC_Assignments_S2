function H = funcionCalcularH(puntos2Dt, puntos2D)
%FUNCIONCALCULARH Summary of this function goes here
%   Detailed explanation goes here

% Number of points
n = size(puntos2Dt, 1);

% Construct the matrix A
A = zeros(2 * n, 9);
for i = 1:n
    x = puntos2Dt(i, 1);
    y = puntos2Dt(i, 2);
    xp = puntos2D(i, 1);
    yp = puntos2D(i, 2);
    A(2*i-1, :) = [x, y, 1, 0, 0, 0, -xp*x, -xp*y, -xp];
    A(2*i, :) = [0, 0, 0, x, y, 1, -yp*x, -yp*y, -yp];
end

% Solve Ah = 0 using SVD
[~, ~, V] = svd(A);
h = V(:, end);

% Reshape h into a 3x3 matrix
H = reshape(h, [3, 3])';

end

