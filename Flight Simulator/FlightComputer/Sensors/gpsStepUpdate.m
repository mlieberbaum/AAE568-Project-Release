function GPS = gpsStepUpdate(GPS, curState, posNoise, velNoise)

    
    % Simple Gaussian Draw
    posTrue = [curState.xE; curState.yE; curState.zE];
    velTrue = [curState.vX; curState.vY; curState.vZ];
    
    GPS.posMsmt = posTrue + posNoise.';
    GPS.velMsmt = velTrue + velNoise.';
    
    
    
end