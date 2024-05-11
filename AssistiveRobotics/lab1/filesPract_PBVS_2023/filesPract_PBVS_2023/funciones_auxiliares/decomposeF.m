function [R1, R2, t1, e0, e1] = decomposeF(k, F)
    %Compute Essential matrix from F

    E = k' * F * k;

    % Perform SVD on E
    [U, ~, V] = svd(E);

    % Ensure proper rotation matrices (det(R) = 1)
    if det(U) < 0
        U = -U;
    end
    if det(V) < 0
        V = -V;
    end

    % Compute possible rotation matrices
    R1 = U * [0, 1, 0; -1, 0, 0; 0, 0, 1] * V';
    R2 = U * [0, -1, 0; 1, 0, 0; 0, 0, 1] * V';

    % Compute translation vector
    t1 = U(:, 3);

    % Epipole in the right image
    e1 = V(:, end);
    e1 = e1/e1(end);
    % Epipole in the left image
    e0 = U(:, end);
    e0 = e0/e0(end);

end

