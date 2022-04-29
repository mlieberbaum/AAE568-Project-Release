function [H, G] = buildPredictionAugmentedMatrices(A, B, C, N)

    rCA = size(C*A,1);
    cCA = size(C*A,2);
    rCAB = size(C*A*B,1);
    cCAB = size(C*A*B,2);
    
    H = zeros(N*rCA, cCA);
    G = zeros(N*rCAB, N*cCAB);
    
    idxrI = 1:rCA:rCA*N;
    idxrF = rCA:rCA:rCA*N;
    
    idxcI = 1:cCAB:cCAB*N;
    idxcF = cCAB:cCAB:cCAB*N;
    
    for idx = 1:N
        
        iI = idxrI(idx);
        iF = idxrF(idx);
        
        H(iI:iF, 1:cCA) = C*A^(idx-1);
        
    end
    
    for idx1 = 1:N  % For each column
        for idx2 = 1:N  % For each row
            
            rI = idxrI(idx2);
            rF = idxrF(idx2);
            cI = idxcI(idx1);
            cF = idxcF(idx1);
            
            if idx1 == idx2
                M = C*B;
            elseif idx1 > idx2
                M = zeros(rCAB,cCAB);
            else
                power = max(idx2 - idx1, 0);
                M = C*(A^power) * B;
            end
            
            G(rI:rF, cI:cF) = M;
            
        end
    end
    
    
end