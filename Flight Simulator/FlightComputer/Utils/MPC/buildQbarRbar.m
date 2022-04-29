function [Qbar, Rbar] = buildQbarRbar(Q, R, N)

    nQ = N;
    Qr = repmat(Q, 1, nQ);
    Qc = mat2cell(Qr, size(Q,1), repmat(size(Q,2),1,nQ));
    Qbar = blkdiag(Qc{:});
    
    nR = N;
    Rr = repmat(R, 1, nR);
    Rc = mat2cell(Rr, size(R,1), repmat(size(R,2),1,nR));
    Rbar = blkdiag(Rc{:});
    
end