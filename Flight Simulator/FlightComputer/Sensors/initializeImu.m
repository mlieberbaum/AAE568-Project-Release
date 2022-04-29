function IMU = initializeImu(imuData)

    IMU.accel.trueBias = imuData.accelBias;
    IMU.gyro.trueBias = imuData.gyroBias;
    
    IMU.accel.specificForceMsmt = nan(3,1);
    IMU.gyro.omegaMsmt = nan(3,1);
    
    IMU.filteredMeas.accel = nan(10,3);
    IMU.filteredMeas.gyro = nan(10,3);
    
end