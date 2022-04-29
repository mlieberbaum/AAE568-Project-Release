function mcParams = generateMCParams(monteCarloFile, nSimPoints, seeds, simtEnd)

    
    % Read input file
    mcInputData = readInputFile(monteCarloFile);
    nRuns = numel(seeds);
    
    
    
    %% BUILD THE MONTE CARLO STRUCTURE
    
    mcParams = struct();
    mcParams(1) = [];
    
    for idx = 1:nRuns
        
        
        % Seet seed
        curSeed = uint32(seeds(idx));
        rng(curSeed);
        
        
        % Atmospheric perturbations
        mcParams(idx).densityMultiplier = mcInputData.densityMultiplierSigma * randn(1,1);
        mcParams(idx).temperatureShift = mcInputData.temperatureShiftSigma * randn(1,1);
        
        
        % Wind model creation
        mcParams(idx).windMultiplier = mcInputData.windSpeedMultiplierSigma * randn(1,1);
        mcParams(idx).windDirection = mcInputData.windDirectionSigma * randn(1,1);
        
        mcParams(idx).gustTimeTable = buildWindGustData(mcInputData.windGustSpeedMultiplierSigma, ...
                                                        mcInputData.averageGustRate, ...
                                                        mcInputData.gustRateMultiplierSigma, ...
                                                        mcInputData.averageGustLength, ...
                                                        mcInputData.gustLengthMultiplierSigma, ...
                                                        mcInputData.gustDirectionOffsetDegSigma, ...
                                                        simtEnd);
        
        
        % IMU Accelerometer and Gyro Biases and Noises
        mcParams(idx).imu.accelBias = mcInputData.imuAccelBiasSigma * randn(3,1);
        mcParams(idx).imu.gyroBias = mcInputData.imuGyroBiasSigma * randn(3,1);
        
        
        % Initial attitude error
        mcParams(idx).initialErrors.attitude = mcInputData.initAttitudeErrorSigma * randn(3,1);
        mcParams(idx).initialErrors.position = mcInputData.initPosErrorSigma * randn(3,1);
        mcParams(idx).initialErrors.velocity = mcInputData.initVelErrorSigma * randn(3,1);
        
        
        % Mass and Inertia Multiplier Scale Factors
        mcParams(idx).pmi.massScaleFactor = mcInputData.massErrorScaleFactorSigma * randn(1,1);
        mcParams(idx).pmi.IxxScaleFactor = mcInputData.IxxErrorScaleFactorSigma * randn(1,1);
        mcParams(idx).pmi.IyyScaleFactor = mcInputData.IyyErrorScaleFactorSigma * randn(1,1);
        mcParams(idx).pmi.IzzScaleFactor = mcInputData.IzzErrorScaleFactorSigma * randn(1,1);
        mcParams(idx).pmi.IxyScaleFactor = mcInputData.IxyErrorScaleFactorSigma * randn(1,1);
        mcParams(idx).pmi.IxzScaleFactor = mcInputData.IxzErrorScaleFactorSigma * randn(1,1);
        mcParams(idx).pmi.IyzScaleFactor = mcInputData.IyzErrorScaleFactorSigma * randn(1,1);
        
        
        % Preallocate random noise for IMU Accel, IMU Gyro, GPS, and Radar
        % Altimeter
        mcParams(idx).imu.accelNoise = mcInputData.imuAccelNoiseSigma * randn(nSimPoints, 3);
        mcParams(idx).imu.gyroNoise = mcInputData.imuGyroNoiseSigma * randn(nSimPoints, 3);
        mcParams(idx).gps.posNoise = mcInputData.gpsPosNoiseSigma * randn(nSimPoints, 3);
        mcParams(idx).gps.velNoise = mcInputData.gpsVelNoiseSigma * randn(nSimPoints, 3);
        mcParams(idx).rdrAlt.posNoise = mcInputData.rdrAltNoiseSigma * randn(nSimPoints, 1);
        
        
        % Preallocate random noise for aoa/beta sensors
        mcParams(idx).aeroSensor.aoaNoise = mcInputData.aoaSensorNoiseSigmaDeg * randn(nSimPoints, 1) * (pi/180);
        mcParams(idx).aeroSensor.betaNoise = mcInputData.betaSensorNoiseSigmaDeg * randn(nSimPoints, 1) * (pi/180);
        
        
        % Aerodynamic error scale factors
        mcParams(idx).aero.mainWingLiftCurveMultiplier = 1 + mcInputData.mainWingLiftCurveMultiplierSigma * randn(1,1);
        mcParams(idx).aero.mainWingMomentCurveMultiplier = 1 + mcInputData.mainWingMomentCurveMultiplierSigma * randn(1,1);
        mcParams(idx).aero.tailLiftCurveMultiplier = 1 + mcInputData.tailLiftCurveMultiplierSigma * randn(1,1);
        mcParams(idx).aero.tailMomentCurveMultiplier = 1 + mcInputData.tailMomentCurveMultiplierSigma * randn(1,1);
        
        
        
    end
    
    
end