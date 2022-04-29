function dataPt = setFCParams(dataPt, FCdt, FCphase, FCSensors, FCNavigation, FCGuidance, FCAutopilot, FCCtrlSurfCmd)

    % Saves off the current state of the flight computer
    dataPt.FC.dt = FCdt;
    dataPt.FC.phase = phase2enum(FCphase);
    
    dataPt.FC.Sensors.truthSensor.xE = FCSensors.truthSensor.xE;
    dataPt.FC.Sensors.truthSensor.yE = FCSensors.truthSensor.yE;
    dataPt.FC.Sensors.truthSensor.zE = FCSensors.truthSensor.zE;
    dataPt.FC.Sensors.truthSensor.xEdot = FCSensors.truthSensor.xEdot;
    dataPt.FC.Sensors.truthSensor.yEdot = FCSensors.truthSensor.yEdot;
    dataPt.FC.Sensors.truthSensor.zEdot = FCSensors.truthSensor.zEdot;
    dataPt.FC.Sensors.truthSensor.u = FCSensors.truthSensor.u;
    dataPt.FC.Sensors.truthSensor.v = FCSensors.truthSensor.v;
    dataPt.FC.Sensors.truthSensor.w = FCSensors.truthSensor.w;
    dataPt.FC.Sensors.truthSensor.phi = FCSensors.truthSensor.phi;
    dataPt.FC.Sensors.truthSensor.theta = FCSensors.truthSensor.theta;
    dataPt.FC.Sensors.truthSensor.psi = FCSensors.truthSensor.psi;
    dataPt.FC.Sensors.truthSensor.p = FCSensors.truthSensor.p;
    dataPt.FC.Sensors.truthSensor.q = FCSensors.truthSensor.q;
    dataPt.FC.Sensors.truthSensor.r = FCSensors.truthSensor.r;
    dataPt.FC.Sensors.truthSensor.aoa = FCSensors.truthSensor.aoa;
    dataPt.FC.Sensors.truthSensor.beta = FCSensors.truthSensor.beta;
    
    dataPt.FC.Sensors.IMU.accel.trueBias = FCSensors.IMU.accel.trueBias;
    dataPt.FC.Sensors.IMU.accel.specificForceMsmt = FCSensors.IMU.accel.specificForceMsmt;
    dataPt.FC.Sensors.IMU.gyro.trueBias = FCSensors.IMU.gyro.trueBias;
    dataPt.FC.Sensors.IMU.gyro.omegaMsmt = FCSensors.IMU.gyro.omegaMsmt;
    dataPt.FC.Sensors.GPS.posMsmt = FCSensors.GPS.posMsmt;
    dataPt.FC.Sensors.GPS.velMsmt = FCSensors.GPS.velMsmt;
    
    dataPt.FC.Navigation.Filters.InertialNavSoln.rhat = FCNavigation.Filters.InertialNavSoln.rhat;
    dataPt.FC.Navigation.Filters.InertialNavSoln.vhat = FCNavigation.Filters.InertialNavSoln.vhat;
    dataPt.FC.Navigation.Filters.InertialNavSoln.rpyhat = FCNavigation.Filters.InertialNavSoln.rpyhat;
    dataPt.FC.Navigation.Filters.IntegratedNavSoln.xhat = FCNavigation.Filters.IntegratedNavSoln.xhat;
    dataPt.FC.Navigation.Filters.IntegratedNavSoln.stdev = FCNavigation.Filters.IntegratedNavSoln.stdev;
    dataPt.FC.Navigation.Filters.IntegratedNavSoln.biasEstimates.accel = FCNavigation.Filters.IntegratedNavSoln.biasEstimates.accel;
    dataPt.FC.Navigation.Filters.IntegratedNavSoln.biasEstimates.gyro = FCNavigation.Filters.IntegratedNavSoln.biasEstimates.gyro;
    dataPt.FC.Navigation.Filters.IntegratedNavSoln.updatePeriod = FCNavigation.Filters.IntegratedNavSoln.updatePeriod;
    dataPt.FC.Navigation.Filters.IntegratedNavSoln.nextUpdateTime = FCNavigation.Filters.IntegratedNavSoln.nextUpdateTime;
    dataPt.FC.Navigation.Filters.IntegratedNavSoln.gpsMeasurement.t = FCNavigation.Filters.IntegratedNavSoln.gpsMeasurement.t;
    dataPt.FC.Navigation.Filters.IntegratedNavSoln.gpsMeasurement.pos = FCNavigation.Filters.IntegratedNavSoln.gpsMeasurement.pos;
    dataPt.FC.Navigation.Filters.IntegratedNavSoln.gpsMeasurement.vel = FCNavigation.Filters.IntegratedNavSoln.gpsMeasurement.vel;
    
    dataPt.FC.Navigation.ControlVariables.crossTrackError = FCNavigation.ControlVariables.crossTrackError;
    dataPt.FC.Navigation.ControlVariables.crossTrackErrorRate = FCNavigation.ControlVariables.crossTrackErrorRate;
    dataPt.FC.Navigation.ControlVariables.LonVars.alpha = FCNavigation.ControlVariables.LonVars.alpha;
    dataPt.FC.Navigation.ControlVariables.LonVars.q = FCNavigation.ControlVariables.LonVars.q;
    dataPt.FC.Navigation.ControlVariables.LonVars.theta = FCNavigation.ControlVariables.LonVars.theta;
    dataPt.FC.Navigation.ControlVariables.LatVars.beta = FCNavigation.ControlVariables.LatVars.beta;
    dataPt.FC.Navigation.ControlVariables.LatVars.p = FCNavigation.ControlVariables.LatVars.p;
    dataPt.FC.Navigation.ControlVariables.LatVars.r = FCNavigation.ControlVariables.LatVars.r;
    dataPt.FC.Navigation.ControlVariables.LatVars.phi = FCNavigation.ControlVariables.LatVars.phi;
    dataPt.FC.Navigation.ControlVariables.airspeed = FCNavigation.ControlVariables.airspeed;
    dataPt.FC.Navigation.ControlVariables.posE = FCNavigation.ControlVariables.posE;
    dataPt.FC.Navigation.ControlVariables.velE = FCNavigation.ControlVariables.velE;
    dataPt.FC.Navigation.ControlVariables.posTaem = FCNavigation.ControlVariables.posTaem;
    dataPt.FC.Navigation.ControlVariables.velTaem = FCNavigation.ControlVariables.velTaem;
    dataPt.FC.Navigation.ControlVariables.rHac = FCNavigation.ControlVariables.rHac;
    dataPt.FC.Navigation.ControlVariables.psiHac = FCNavigation.ControlVariables.psiHac;
    dataPt.FC.Navigation.ControlVariables.rdotHac = FCNavigation.ControlVariables.rdotHac;
    
    dataPt.FC.Guidance.Gains.Vnav.KP = FCGuidance.Gains.Vnav.KP;
    dataPt.FC.Guidance.Gains.Vnav.KI = FCGuidance.Gains.Vnav.KI;
    dataPt.FC.Guidance.Gains.Vnav.KD = FCGuidance.Gains.Vnav.KD;
    dataPt.FC.Guidance.Gains.Lnav.KP = FCGuidance.Gains.Lnav.KP;
    dataPt.FC.Guidance.Gains.Lnav.KI = FCGuidance.Gains.Lnav.KI;
    dataPt.FC.Guidance.Gains.Lnav.KD = FCGuidance.Gains.Lnav.KD;
    
    dataPt.FC.Guidance.Commands.altitude = FCGuidance.Commands.altitude;
    dataPt.FC.Guidance.Commands.theta = FCGuidance.Commands.theta;
    dataPt.FC.Guidance.Commands.phi = FCGuidance.Commands.phi;
    dataPt.FC.Guidance.Commands.airspeed = FCGuidance.Commands.airspeed;
    dataPt.FC.Guidance.Commands.spdbk = FCGuidance.Commands.spdbk;
    dataPt.FC.Guidance.Commands.spdbkControllerLevel = FCGuidance.Commands.spdbkControllerLevel;
    
    dataPt.FC.Guidance.Integrators.altError = FCGuidance.Integrators.altError;
    dataPt.FC.Guidance.Integrators.crossTrackError = FCGuidance.Integrators.crossTrackError;
    
    dataPt.FC.Autopilot.Gains.Vnav.K = FCAutopilot.Gains.Vnav.K;
    dataPt.FC.Autopilot.Gains.Vnav.H = FCAutopilot.Gains.Vnav.H;
    dataPt.FC.Autopilot.Gains.Lnav.K = FCAutopilot.Gains.Lnav.K;
    dataPt.FC.Autopilot.Gains.Lnav.H = FCAutopilot.Gains.Lnav.H;
    
    dataPt.FC.Autopilot.Integrators.thetaError = FCAutopilot.Integrators.thetaError;
    dataPt.FC.Autopilot.Integrators.phiError = FCAutopilot.Integrators.phiError;
    
    dataPt.FC.ControlSurfCommand.gear = FCCtrlSurfCmd.gear;
    dataPt.FC.ControlSurfCommand.trim = FCCtrlSurfCmd.trim;
    dataPt.FC.ControlSurfCommand.elevator = FCCtrlSurfCmd.elevator;
    dataPt.FC.ControlSurfCommand.aileron = FCCtrlSurfCmd.aileron;
    dataPt.FC.ControlSurfCommand.rudder = FCCtrlSurfCmd.rudder;
    dataPt.FC.ControlSurfCommand.spdbk = FCCtrlSurfCmd.spdbk;
    
end