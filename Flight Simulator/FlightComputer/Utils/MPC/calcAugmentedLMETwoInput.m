function [E, W] = calcAugmentedLMETwoInput(N, uCur, uBound, xk, ylim, A, C, G, dulim)

    % Compute linear matrix inequality matrices E and W
    
    % Build S matrix
    S = zeros(N*2,N*2);
    for i = 1:2*N
        for j = 1:2*N
            if j <= i && mod(i-2-j,2) == 0
                S(i,j) = 1;
            end
        end
    end
    
    E = [ S
         -S];
    
    W = zeros(4*N,1);
    idxHalf = 2*N;
    
    W(1:2:idxHalf,1) = uBound - uCur(1);
    W(2:2:idxHalf,1) = uBound - uCur(2);
    
    W(idxHalf+1:2:end,1) = uBound + uCur(1);
    W(idxHalf+2:2:end,1) = uBound + uCur(2);
    
    
    % Compute output constraints
    Ahat = [];
    
    for i = 1:N
        Ahat = [Ahat; C*A^i];
    end
    
    yhat = repmat(ylim, N, 1);
    
    
    % Input rate constraints
    Edu = [eye(2*N)
           -eye(2*N)];
    Wdu = [dulim*ones(2*N,1)
           dulim*ones(2*N,1)];
    
    
    E = [E
         Edu
         G
         -G];
    
    W = [W
         Wdu
         yhat - Ahat*xk
         yhat + Ahat*xk];
    
end