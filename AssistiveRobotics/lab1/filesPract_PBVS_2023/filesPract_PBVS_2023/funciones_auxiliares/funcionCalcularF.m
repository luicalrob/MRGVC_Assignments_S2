function F = funcionCalcularF(puntos2Dt, puntos2D)
%FUNCIONCALCULARF Estimate the Fundamental matrix from matching points

% Number of points
n = size(puntos2Dt, 1);

% Construct the matrix A
A = zeros(n, 9);
for i = 1:n
    xp = puntos2Dt(i, 1);
    yp = puntos2Dt(i, 2);
    x = puntos2D(i, 1);
    y = puntos2D(i, 2);
    A(i, :) = [x*xp, y*xp, xp, x*yp, y*yp, yp, x, y, 1];
end

% Solve Af = 0 using SVD
[~, ~, V] = svd(A);
f = V(:, end);

% Reshape f into a 3x3 matrix and enforce rank-2 constraint
F = reshape(f, [3, 3])';

% % Enforce rank-2 constraint on F
[U, S, V] = svd(F);
S(3,3) = 0; % Set smallest singular value to zero
F = U * S * V';

end

