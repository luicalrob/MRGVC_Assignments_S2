function [R1, R2, t1, t2] = decomposeH(K, H, d)
    Hc = inv(K) * H * K;

    % Singular value decomposition of Hc
    [U,S,V] = svd(Hc);

    Hc = Hc./S(2,2);
    v1 = V(:, 1);
    v2 = V(:, 2);
    v3= V(:, 3);

    % Compute alpha values using lambda values from S matrix diagonal elements 
    S = S.^2;
    lambda1 = S(1,1);
    lambda2 = S(2,2);
    lambda3 = S(3,3);

    alpha = sqrt((lambda3-lambda2)/(lambda3-lambda1));
    beta = sqrt((lambda2-lambda1)/(lambda3-lambda1));
    
    % Compute omega values using alpha values and vectors v 
    w1= alpha * v1 + beta * v3;
    w2= alpha * v1 - beta * v3;

    U1 = [w1, v2, cross(w1,v2)];
    U2 = [w2, v2, cross(w2,v2)];

    W1 = [Hc*w1, Hc*v2, cross(Hc*w1,Hc*v2)];
    W2 = [Hc*w2, Hc*v2, cross(Hc*w2,Hc*v2)];
    
    % Solutions for R and t using omega values 
    R1= W1*U1';
    n1= cross(w1,v2);
    t1=(eye(3) - R1'*Hc)*n1*d;
    
    R2= W2*U2';
    n2= cross(w2,v2);
    t2=(eye(3) - R2'*Hc)*n2*d;
end

