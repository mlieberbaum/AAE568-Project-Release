function data = simDataPoint(npts)

    % Preallocation of sim data structure
    data.t = nan(npts, 1);
    
    data.aero.rho = nan(npts, 1);
    data.aero.mach = nan(npts, 1);    
    data.aero.airspeed = nan(npts, 1);
    data.aero.Tinf = nan(npts, 1);
    
    
    % Control Surfaces
    data.ctrlSurf.gear = nan(npts, 1);
    data.ctrlSurf.spdbk = nan(npts, 1);
    data.ctrlSurf.elevator = nan(npts, 1);
    data.ctrlSurf.aileron = nan(npts, 1);
    data.ctrlSurf.rudder = nan(npts, 1);
    data.ctrlSurf.trim = nan(npts, 1);
    
    
    % True state and derivative of true state
    data.state.u = nan(npts, 1);
    data.state.v = nan(npts, 1);
    data.state.w = nan(npts, 1);
    data.state.p = nan(npts, 1);
    data.state.q = nan(npts, 1);
    data.state.r = nan(npts, 1);
    data.state.psi = nan(npts, 1);
    data.state.theta = nan(npts, 1);
    data.state.phi = nan(npts, 1);
    data.state.xE = nan(npts, 1);
    data.state.yE = nan(npts, 1);
    data.state.zE = nan(npts, 1);
    data.state.vX = nan(npts, 1);
    data.state.vY = nan(npts, 1);
    data.state.vZ = nan(npts, 1);
    data.state.aX = nan(npts, 1);
    data.state.aY = nan(npts, 1);
    data.state.aZ = nan(npts, 1);
    
    
    % Forces and moments
    data.forceMoment.netForceVecInB = nan(npts, 3);
    data.forceMoment.liftVecInB = nan(npts, 3);
    data.forceMoment.dragVecInB = nan(npts, 3);
    data.forceMoment.weightVecInB = nan(npts, 3);
    data.forceMoment.momentVecInB = nan(npts, 3);
    
    
    % Wind
    data.wind.windNED = nan(npts, 3);
    data.wind.windUVW = nan(npts, 3);
    
    
    % Flight Computer
    data.FC.dt = nan(npts, 1);
    data.FC.phase = nan(npts, 1);
    
    data.FC.Sensors.truthSensor.xE = nan(npts, 1);
    data.FC.Sensors.truthSensor.yE = nan(npts, 1);
    data.FC.Sensors.truthSensor.zE = nan(npts, 1);
    data.FC.Sensors.truthSensor.xEdot = nan(npts, 1);
    data.FC.Sensors.truthSensor.yEdot = nan(npts, 1);
    data.FC.Sensors.truthSensor.zEdot = nan(npts, 1);
    data.FC.Sensors.truthSensor.u = nan(npts, 1);
    data.FC.Sensors.truthSensor.v = nan(npts, 1);
    data.FC.Sensors.truthSensor.w = nan(npts, 1);
    data.FC.Sensors.truthSensor.phi = nan(npts, 1);
    data.FC.Sensors.truthSensor.theta = nan(npts, 1);
    data.FC.Sensors.truthSensor.psi = nan(npts, 1);
    data.FC.Sensors.truthSensor.p = nan(npts, 1);
    data.FC.Sensors.truthSensor.q = nan(npts, 1);
    data.FC.Sensors.truthSensor.r = nan(npts, 1);
    data.FC.Sensors.truthSensor.aoa = nan(npts, 1);
    data.FC.Sensors.truthSensor.beta = nan(npts, 1);
    
    data.FC.Sensors.IMU.accel.trueBias = nan(npts, 3);
    data.FC.Sensors.IMU.accel.specificForceMsmt = nan(npts, 3);
    data.FC.Sensors.IMU.gyro.trueBias = nan(npts, 3);
    data.FC.Sensors.IMU.gyro.omegaMsmt = nan(npts, 3);
    data.FC.Sensors.GPS.posMsmt = nan(npts, 3);
    data.FC.Sensors.GPS.velMsmt = nan(npts, 3);
    
    data.FC.Navigation.Filters.InertialNavSoln.rhat = nan(npts, 3);
    data.FC.Navigation.Filters.InertialNavSoln.vhat = nan(npts, 3);
    data.FC.Navigation.Filters.InertialNavSoln.rpyhat = nan(npts, 3);
    data.FC.Navigation.Filters.IntegratedNavSoln.xhat = nan(npts, 15);
    data.FC.Navigation.Filters.IntegratedNavSoln.stdev = nan(npts, 15);
    data.FC.Navigation.Filters.IntegratedNavSoln.biasEstimates.accel = nan(npts, 3);
    data.FC.Navigation.Filters.IntegratedNavSoln.biasEstimates.gyro = nan(npts, 3);
    data.FC.Navigation.Filters.IntegratedNavSoln.updatePeriod = nan(npts, 1);
    data.FC.Navigation.Filters.IntegratedNavSoln.nextUpdateTime = nan(npts, 1);
    data.FC.Navigation.Filters.IntegratedNavSoln.gpsMeasurement.t = nan(npts, 1);
    data.FC.Navigation.Filters.IntegratedNavSoln.gpsMeasurement.pos = nan(npts, 3);
    data.FC.Navigation.Filters.IntegratedNavSoln.gpsMeasurement.vel = nan(npts, 3);
    
    data.FC.Navigation.ControlVariables.crossTrackError = nan(npts, 1);
    data.FC.Navigation.ControlVariables.crossTrackErrorRate = nan(npts, 1);
    data.FC.Navigation.ControlVariables.LonVars.alpha = nan(npts, 1);
    data.FC.Navigation.ControlVariables.LonVars.q = nan(npts, 1);
    data.FC.Navigation.ControlVariables.LonVars.theta = nan(npts, 1);
    data.FC.Navigation.ControlVariables.LatVars.beta = nan(npts, 1);
    data.FC.Navigation.ControlVariables.LatVars.p = nan(npts, 1);
    data.FC.Navigation.ControlVariables.LatVars.r = nan(npts, 1);
    data.FC.Navigation.ControlVariables.LatVars.phi = nan(npts, 1);
    data.FC.Navigation.ControlVariables.airspeed = nan(npts, 1);
    data.FC.Navigation.ControlVariables.posE = nan(npts, 3);
    data.FC.Navigation.ControlVariables.velE = nan(npts, 3);
    data.FC.Navigation.ControlVariables.posTaem = nan(npts, 3);
    data.FC.Navigation.ControlVariables.velTaem = nan(npts, 3);
    data.FC.Navigation.ControlVariables.rHac = nan(npts, 1);
    data.FC.Navigation.ControlVariables.psiHac = nan(npts, 1);
    data.FC.Navigation.ControlVariables.rdotHac = nan(npts, 1);
    
    data.FC.Guidance.Gains.Vnav.KP = nan(npts, 1);
    data.FC.Guidance.Gains.Vnav.KI = nan(npts, 1);
    data.FC.Guidance.Gains.Vnav.KD = nan(npts, 1);
    data.FC.Guidance.Gains.Lnav.KP = nan(npts, 1);
    data.FC.Guidance.Gains.Lnav.KI = nan(npts, 1);
    data.FC.Guidance.Gains.Lnav.KD = nan(npts, 1);
    data.FC.Guidance.Commands.altitude = nan(npts, 1);
    data.FC.Guidance.Commands.theta = nan(npts, 1);
    data.FC.Guidance.Commands.phi = nan(npts, 1);
    data.FC.Guidance.Commands.airspeed = nan(npts, 1);
    data.FC.Guidance.Commands.spdbk = nan(npts, 1);
    data.FC.Guidance.Commands.spdbkControllerLevel = nan(npts, 1);
    data.FC.Guidance.Integrators.altError = nan(npts, 1);
    data.FC.Guidance.Integrators.crossTrackError = nan(npts, 1);
    
    data.FC.Autopilot.Gains.Vnav.K = nan(npts, 3);
    data.FC.Autopilot.Gains.Vnav.H = nan(npts, 1);
    data.FC.Autopilot.Gains.Lnav.K = nan(npts, 4);
    data.FC.Autopilot.Gains.Lnav.H = nan(npts, 1);
    data.FC.Autopilot.Integrators.thetaError = nan(npts, 1);
    data.FC.Autopilot.Integrators.phiError = nan(npts, 1);
    
    data.FC.ControlSurfCommand.gear = nan(npts, 1);
    data.FC.ControlSurfCommand.trim = nan(npts, 1);
    data.FC.ControlSurfCommand.elevator = nan(npts, 1);
    data.FC.ControlSurfCommand.aileron = nan(npts, 1);
    data.FC.ControlSurfCommand.rudder = nan(npts, 1);
    data.FC.ControlSurfCommand.spdbk = nan(npts, 1);

    
end