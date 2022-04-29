function FC = initializeFlightComputer(trajectoryInputFile, dt, sensorMode, rwyHeading, fcPhase, ...
    initialErrors, imuData, initialState, flightComputerInputParams, spaceplane, lateralAutopilotMode, ...
    longitudinalAutopilotMode, speedbrakeAutopilotMode)

    
    %% DATA
    FC.dt = dt;
    
    fcPhases = {'TAEM', 'HAC', 'LONG_FINAL', 'PREFLARE', 'SHORT_FINAL', 'FLARE'};
    
    if ~any(contains(fcPhases, fcPhase))
        error('Invalid FC Phase of Flight Input');
    end
    
    FC.phase = fcPhase;
    FC.rwyHeading = rwyHeading;
    
    FC.count = 0;
    
    if strcmp(sensorMode, 'TRUTH')
        FC.useTruth = 1;
    else
        FC.useTruth = 0;
    end
    
    FC.spaceplane = spaceplane;
    
    if strcmp(lateralAutopilotMode, 'MPC')
        FC.useLateralMPC = 1;
    else
        FC.useLateralMPC = 0;
    end
    
    if strcmp(longitudinalAutopilotMode, 'MPC')
        FC.useLongitudinalMPC = 1;
    else
        FC.useLongitudinalMPC = 0;
    end
    
    if strcmp(speedbrakeAutopilotMode, 'MPC')
        FC.useSpeedbrakeMPC = 1;
    else
        FC.useSpeedbrakeMPC = 0;
    end
    
    
    
    %% TRAJECTORY PROPERTIES
    FC.trajectoryProps = load(trajectoryInputFile);
    FC.trajectoryProps = FC.trajectoryProps.trajectoryParameters;
    
    
    
    %% SENSORS
    FC.Sensors.truthSensor = struct();
    FC.Sensors.IMU = initializeImu(imuData);
    FC.Sensors.GPS = initializeGps();
    FC.Sensors.rdrAlt.msmt = [];
    FC.Sensors.rdrAlt.state = 0;
    FC.Sensors.aoaSensor = nan(25,1);
    FC.Sensors.betaSensor = nan(25,1);
    
    
    
    %% NAV
    
    % Filtered estimates
    [FC.Navigation.Filters.InertialNavSoln, FC.Navigation.Filters.IntegratedNavSoln] = ...
        initializeNavSolution(initialErrors, initialState, flightComputerInputParams);
    FC.Navigation.Filters.qinf.State = [];
    FC.Navigation.Filters.qinf.Variance = [];
    FC.Navigation.Filters.aoa.State = [];
    FC.Navigation.Filters.aoa.Variance = [];
    FC.Navigation.Filters.beta.State = [];
    FC.Navigation.Filters.beta.Variance = [];
    
    
    % Control state variables
    FC.Navigation.ControlVariables.crossTrackError = 0;
    FC.Navigation.ControlVariables.crossTrackErrorRate = 0;
    FC.Navigation.ControlVariables.altitudeError = 0;
    FC.Navigation.ControlVariables.altitudeErrorRate = 0;
    FC.Navigation.ControlVariables.airspeed = 0;
    FC.Navigation.ControlVariables.posE = 0;
    FC.Navigation.ControlVariables.velE = 0;
    FC.Navigation.ControlVariables.posEprev = 0;
    FC.Navigation.ControlVariables.velEprev = 0;
    FC.Navigation.ControlVariables.posTaem = 0;
    FC.Navigation.ControlVariables.velTaem = 0;
    FC.Navigation.ControlVariables.rHac = 0;
    FC.Navigation.ControlVariables.psiHac = 0;
    FC.Navigation.ControlVariables.rdotHac = 0;
    
    
    % Control longitudinal and lateral variables
    FC.Navigation.ControlVariables.LonVars.alpha = 0;
    FC.Navigation.ControlVariables.LonVars.q = 0;
    FC.Navigation.ControlVariables.LonVars.theta = 0;
    
    FC.Navigation.ControlVariables.LatVars.beta = 0;
    FC.Navigation.ControlVariables.LatVars.p = 0;
    FC.Navigation.ControlVariables.LatVars.r = 0;
    FC.Navigation.ControlVariables.LatVars.phi = 0;

    
    
    %% GUIDANCE    
    FC.Guidance.Gains.Vnav.KP = 0.0025;
    FC.Guidance.Gains.Vnav.KI = 0.001;
    FC.Guidance.Gains.Vnav.KD = 0.01;
    
    FC.Guidance.Gains.Lnav.KP = 0.001;
    FC.Guidance.Gains.Lnav.KI = 0.0;
    FC.Guidance.Gains.Lnav.KD = 0.02;
    
    FC.Guidance.Commands.altitude = 0;
    FC.Guidance.Commands.theta = initialState.theta;
    FC.Guidance.Commands.gammaNominal = 0;
    FC.Guidance.Commands.phi = 0;
    FC.Guidance.Commands.spdbk = 0;
    FC.Guidance.Commands.spdbkControllerLevel = 0;
    FC.Guidance.Commands.airspeed = 0;
    
    FC.Guidance.Integrators.altError = 0;
    FC.Guidance.Integrators.crossTrackError = 0;
    
    FC.Guidance.CommandRateLimits.phi = 6 * pi/180;
    FC.Guidance.CommandRateLimits.theta = 10 * pi/180;
    
        
    
    %% AUTOPILOT
    FC.Autopilot.updatePeriod = 1 / flightComputerInputParams.autopilot.updateRate;
    FC.Autopilot.nextUpdateTime = 0;
    
    FC.Autopilot.Gains.Vnav.K = spaceplane.longitudinalGains.K;
    FC.Autopilot.Gains.Vnav.H = spaceplane.longitudinalGains.H;
    
    FC.Autopilot.Gains.Lnav.K = spaceplane.lateralGains.K;
    FC.Autopilot.Gains.Lnav.H = spaceplane.lateralGains.H;
    
    FC.Autopilot.Integrators.thetaError = 0;
    FC.Autopilot.Integrators.phiError = 0;
    
    FC.Autopilot.MPC.Lateral.xTilde = zeros(6,1);
    FC.Autopilot.MPC.Lateral.xPrev = zeros(4,1);
    FC.Autopilot.MPC.Lateral.curU = [0; 0];
    
    FC.Autopilot.MPC.Longitudinal.xTilde = zeros(4,1);
    FC.Autopilot.MPC.Longitudinal.xTilde(4) = initialState.theta;
    FC.Autopilot.MPC.Longitudinal.xPrev = zeros(3,1);
    FC.Autopilot.MPC.Longitudinal.xPrev(3) = initialState.theta;
    FC.Autopilot.MPC.Longitudinal.curU = 0;
    
    FC.Autopilot.MPC.Speedbrake.xTilde = zeros(2,1);
    FC.Autopilot.MPC.Speedbrake.xTilde(2) = norm([initialState.u, initialState.v, initialState.w]);
    FC.Autopilot.MPC.Speedbrake.xPrev = norm([initialState.u, initialState.v, initialState.w]);
    FC.Autopilot.MPC.Speedbrake.curU = 0;
    
    FC.Autopilot.Gains.Speedbrake.K = 0.1;
    
    FC.Autopilot.MPC.maxCtrlSurfDeflectionDelta = flightComputerInputParams.autopilot.maxCtrlSurfDeflectionDelta;
    FC.Autopilot.MPC.pitchLimit = flightComputerInputParams.autopilot.pitchLimitDeg * pi/180;
    FC.Autopilot.MPC.rollLimit = flightComputerInputParams.autopilot.rollLimitDeg * pi/180;
    FC.Autopilot.MPC.sideslipLimit = flightComputerInputParams.autopilot.sideslipLimitDeg * pi/180;
    
    
    
    %% CONTROL SURFACE COMMANDS
    FC.ControlSurfCommand = zeroCtrlSurf();
    
    
    
end