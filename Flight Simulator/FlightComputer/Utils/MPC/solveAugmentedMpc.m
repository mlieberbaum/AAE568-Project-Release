function du = solveAugmentedMpc(A, B, C, N, Q, R, x, uCur, uUpperBound, uLowerBound, xTgt, pitchLimitRad, duLimit)
    
    
    % Build prediction matrices
    [H, G] = buildPredictionAugmentedMatrices(A, B, C, N);


    % Build linear matrix inequalities
    [E, W] = calcAugmentedLME(N, uCur, uUpperBound, uLowerBound, x, pitchLimitRad, A, C, G, duLimit);
    
    
    % Build Qbar and Rbar
    [Qbar, Rbar] = buildQbarRbar(Q(1,1), R, N);
    
    
    % Build L and F
    L = G.'*Qbar*G + Rbar;
    F = G.'*Qbar*H*A;

    
    % With constraints
    L = chol(L,'lower');
    Linv = inv(L);
    iA = false(size(W));
    opt = mpcActiveSetOptions;
    opt.IntegrityChecks = false;
    opt.UseHessianAsInput = false;
    [u,status] = mpcActiveSetSolver(Linv,F*(x-xTgt),E,W,[],[],iA,opt);
    
    if status <= 0
        warning('MPC Issue');
    end
    
    du = u(1);
    
    
    
end