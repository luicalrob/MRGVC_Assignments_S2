function [q_inverse,error] = inverseKinematics2(limb, T, q_initial1, q_initial2)

try
    [q_inverse1, error1] = ikcon(limb,T, q_initial1);

catch
    fprintf('[LM] Solution 1 failed.\n');
    error1 = inf;
end

try
    [q_inverse2, error2] = ikcon(limb,T, q_initial2);
catch
    fprintf('[LM] Solution 2 failed.\n');
    error2 = inf;
end

% Select the solution with the smallest error
if error1 < error2
    q_inverse = q_inverse1;
    error = error1;
else
    q_inverse = q_inverse2;
    error = error2;
end

fprintf('Selected solution with error %f.\n', error);

end
