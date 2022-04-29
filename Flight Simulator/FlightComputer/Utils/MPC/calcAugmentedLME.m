function [E, W] = calcAugmentedLME(N, uCur, uUpperBound, uLowerBound, xk, ylim, A, C, G, dulim)

    % Compute linear matrix inequality matrices E and W for simple actuator
    % position bounds
    
    % Build S matrix
    S = zeros(N);
    for i = 1:N
        for j = 1:N
            if j <= i
                S(i,j) = 1;
            end
        end
    end
    c = ones(N,1);
    
    % Compute output constraints
    Ahat = [];
    
    for i = 1:N
        Ahat = [Ahat; C*A^i];
    end
    
    yhat = repmat(ylim, N, 1);
    
    
    E = [ S
         -S
          eye(N)
         -eye(N)
          G
         -G];
    
    W = [ uUpperBound - c*uCur
          uLowerBound + c*uCur
          dulim*ones(N,1)
          dulim*ones(N,1)
          yhat - Ahat*xk
          yhat + Ahat*xk];
    
end