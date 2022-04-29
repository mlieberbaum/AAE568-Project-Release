function R = repairMatrix(Rin)

    [U,~,V] = svd(Rin);
    R = U * V.';
    
end