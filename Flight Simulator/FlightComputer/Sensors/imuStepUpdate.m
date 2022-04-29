function IMU = imuStepUpdate(IMU, curState, imuAccelNoise, imuGyroNoise)

    
    % True DCM and altitude
    Cib_true = Tearth2body(curState.phi, curState.theta, curState.psi);
    altTrue = -curState.zE;
    
    
    % True ravity vector in inertial frame
    gTrue = 398600439968871 / ((altTrue + 6371010.0)^2);
    gammaTrue_i = [0; 0; gTrue];
    
    
    % True acceleration in the inertial frame
    aTrue_i = [curState.aX; curState.aY; curState.aZ];
    
    
    % True specific force in the inertial frame
    fTrue_i = aTrue_i - gammaTrue_i;
    
    
    % True specific force in the body frame
    fTrue_b = Cib_true * fTrue_i;
    
    
    % Accelerometer Measurements
    IMU.accel.specificForceMsmt = fTrue_b + IMU.accel.trueBias + imuAccelNoise.';
    
    
    % Gyro Measurements
    wbTrue = [curState.p; curState.q; curState.r];
    IMU.gyro.omegaMsmt = wbTrue + IMU.gyro.trueBias + imuGyroNoise.';
    
    
    % Filtered gyro measurements
    IMU.filteredMeas.gyro(1:9,:) = IMU.filteredMeas.gyro(2:10,:);
    IMU.filteredMeas.gyro(10,:) = IMU.gyro.omegaMsmt';
    
    
end