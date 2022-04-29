function mcParams = zeroMcPerturbations(monteCarloFile, nSimPoints)


    rng('default');
    
    % Read input file
    mcInputData = readInputFile(monteCarloFile);
    
    
    % Atmospheric perturbations
    mcParams.densityMultiplier = 0;
    mcParams.temperatureShift = 0;
    
    
    % Wind
    mcParams.windMultiplier = 0;
    mcParams.windDirection = 0;
    mcParams.gustTimeTable = [];


    % IMU Accelerometer and Gyro Biases and Noises
    mcParams.imu.accelBias = [0.0005376671395461
                              0.00183388501459509
                             -0.00225884686100365];
    mcParams.imu.gyroBias = [4.3108666018406e-08
                             1.5938261992949e-08
                            -6.53844148152637e-08];


    % Initial attitude error
    mcParams.initialErrors.attitude = [.01; .01; .01];
    mcParams.initialErrors.position = [2; 3; 5];
    mcParams.initialErrors.velocity = [-.08; .04; .12];
        
        
    % Mass and Inertia Multiplier Scale Factors
    mcParams.pmi.massScaleFactor = 0;
    mcParams.pmi.IxxScaleFactor = 0;
    mcParams.pmi.IyyScaleFactor = 0;
    mcParams.pmi.IzzScaleFactor = 0;
    mcParams.pmi.IxyScaleFactor = 0;
    mcParams.pmi.IxzScaleFactor = 0;
    mcParams.pmi.IyzScaleFactor = 0;


    % Preallocate random noise for IMU Accel, IMU Gyro, and GPS
    mcParams.imu.accelNoise = mcInputData.imuAccelNoiseSigma * randn(nSimPoints, 3);
    mcParams.imu.gyroNoise = mcInputData.imuGyroNoiseSigma * randn(nSimPoints, 3);
    mcParams.gps.posNoise = mcInputData.gpsPosNoiseSigma * randn(nSimPoints, 3);
    mcParams.gps.velNoise = mcInputData.gpsVelNoiseSigma * randn(nSimPoints, 3);
    mcParams.rdrAlt.posNoise = mcInputData.rdrAltNoiseSigma * randn(nSimPoints, 1);
    
    
    % Preallocate random noise for aoa/beta sensors
    mcParams.aeroSensor.aoaNoise = mcInputData.aoaSensorNoiseSigmaDeg * randn(nSimPoints, 1) * (pi/180);
    mcParams.aeroSensor.betaNoise = mcInputData.betaSensorNoiseSigmaDeg * randn(nSimPoints, 1) * (pi/180);
    
    
    % Aero properties error
    mcParams.aero.mainWingLiftCurveMultiplier = 1;
    mcParams.aero.mainWingMomentCurveMultiplier = 1;
    mcParams.aero.tailLiftCurveMultiplier = 1;
    mcParams.aero.tailMomentCurveMultiplier = 1;

    
end