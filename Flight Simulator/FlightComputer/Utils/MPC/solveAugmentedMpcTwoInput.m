function du = solveAugmentedMpcTwoInput(A, B, C, N, Q, R, x, uCur, uBound, xTgt, ylimits, dulim)
    
    
    % Build prediction matrices
    [H, G] = buildPredictionAugmentedMatrices(A, B, C, N);


    % Build linear matrix inequalities
    [E, W] = calcAugmentedLMETwoInput(N, uCur, uBound, x, ylimits, A, C, G, dulim);
    
    
    % Build Qbar and Rbar
    [Qbar, Rbar] = buildQbarRbar(Q(1:2,1:2), R, N);
    
    
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
    
    du = u(1:2);
    
    
end