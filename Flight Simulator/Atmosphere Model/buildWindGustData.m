function gustTimeTable = buildWindGustData(windGustSpeedMultiplierSigma, averageGustRate, gustRateMultiplierSigma, averageGustLength, gustLengthMultiplierSigma, gustDirectionOffsetDegSigma, simtEnd)

    % Build wind gust data
    idxWrite = 1;
    gustTimeTable = [];
    
    t = 0;
    
    while t < simtEnd
        
        % Next gust time
        gustTimeStart = t + (60 / (averageGustRate + gustRateMultiplierSigma * randn(1,1)));
        gustLength = averageGustLength + gustLengthMultiplierSigma * randn(1,1);
        gustMaxSpeedFactor = abs(windGustSpeedMultiplierSigma * randn(1,1));
        gustDirectionOffset = gustDirectionOffsetDegSigma * randn(1,1);
        
        gustTimeTable(idxWrite, 1) = idxWrite;
        gustTimeTable(idxWrite, 2) = gustTimeStart;
        gustTimeTable(idxWrite, 3) = gustTimeStart + gustLength;
        gustTimeTable(idxWrite, 4) = gustMaxSpeedFactor;
        gustTimeTable(idxWrite, 5) = gustDirectionOffset;
        
        t = gustTimeStart;
        idxWrite = idxWrite + 1;
        
    end
    
    
end