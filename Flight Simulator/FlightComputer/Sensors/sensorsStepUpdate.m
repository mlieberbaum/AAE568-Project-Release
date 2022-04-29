function FC = sensorsStepUpdate(FC, curState, randomNoise)

    
    %% TRUTH SENSOR
    
    % In truth mode, only the truth sensor is updated
    FC.Sensors.truthSensor.xE = curState.xE;
    FC.Sensors.truthSensor.yE = curState.yE;
    FC.Sensors.truthSensor.zE = curState.zE;
    FC.Sensors.truthSensor.xEdot = curState.vX;
    FC.Sensors.truthSensor.yEdot = curState.vY;
    FC.Sensors.truthSensor.zEdot = curState.vZ;
    FC.Sensors.truthSensor.u = curState.u;
    FC.Sensors.truthSensor.v = curState.v;
    FC.Sensors.truthSensor.w = curState.w;

    FC.Sensors.truthSensor.phi = curState.phi;
    FC.Sensors.truthSensor.theta = curState.theta;
    FC.Sensors.truthSensor.psi = curState.psi;
    FC.Sensors.truthSensor.p = curState.p;
    FC.Sensors.truthSensor.q = curState.q;
    FC.Sensors.truthSensor.r = curState.r;

    FC.Sensors.truthSensor.aoa = atan(curState.w / curState.u);
    FC.Sensors.truthSensor.beta = atan(curState.v / curState.u);
    
    
    %% IMU
    FC.Sensors.IMU = imuStepUpdate(FC.Sensors.IMU, curState, randomNoise.imuAccel, randomNoise.imuGyro);
    
    
    
    %% GPS
    FC.Sensors.GPS = gpsStepUpdate(FC.Sensors.GPS, curState, randomNoise.gpsPos, randomNoise.gpsVel);
    
    
    
    %% AOA & BETA SENSORS
    FC.Sensors.aoaSensor(1:24,1) = FC.Sensors.aoaSensor(2:25,1);
    FC.Sensors.betaSensor(1:24,1) = FC.Sensors.betaSensor(2:25,1);
    FC.Sensors.aoaSensor(25,1) = FC.Sensors.truthSensor.aoa + randomNoise.aoa;
    FC.Sensors.betaSensor(25,1) = FC.Sensors.truthSensor.beta + randomNoise.beta;
    
    
    %% RADAR ALTIMETER
    FC.Sensors.rdrAlt.msmt = curState.zE + randomNoise.rdrAlt;
    if FC.Sensors.rdrAlt.state == 0 && -FC.Sensors.GPS.posMsmt(3) < 500
        FC.Sensors.rdrAlt.state = 1;
    end
    
    
end