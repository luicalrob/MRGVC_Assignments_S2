function q_inverse = inverseKinematics(limb, T, q_initial1, q_initial2, qr)

tol = 1e-6;
tol_transpose = 1e-2;
rlimit = 1e3;
ilimit = 1e6;
lambda = 1e-2;
lambdamin = 1e-4;
%For transpose method
stepsize = 5e-2;

try
    q_inverse = ikine(limb,T, 'q0', q_initial1, 'ilimit', ilimit, ...
        'tol', tol,'rlimit', rlimit, 'lambda', lambda, 'lambdamin', lambdamin, 'verbose');
catch
    fprintf('[LM] Solution 1 failed.\n');
    try
        q_inverse = ikine(limb,T, 'q0', q_initial2, 'ilimit', ilimit, ...
            'tol', tol,'rlimit', rlimit, 'lambda', lambda, 'lambdamin', lambdamin, 'verbose');
    catch
        fprintf('[LM] Solution 2 failed. Trying transpose\n');
        q_inverse = ikine(limb,T, 'q0',q_initial2, 'ilimit', ilimit, ...
        'tol', tol_transpose,'rlimit', rlimit, 'lambda', lambda, 'lambdamin', lambdamin, 'transpose', stepsize, 'verbose');
    end
end