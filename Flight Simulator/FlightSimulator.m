function FlightSimulator(simulationInputFile, mcParams, saveData, createPlots, printProgress, runId, outputFolder)

% =========================================================================
% 
%                      SPACE PLANE FLIGHT SIMULATOR
%
% Full six degree-of-freedom flight simulator for a winged vehicle.
%
% =========================================================================




%% INITIALIZATION

% Load input data
inputData = loadInputData(simulationInputFile);
perfectSpaceplane = loadSpaceplane(inputData.spaceplaneInputPath, inputData.spaceplaneInputFile);
flightComputerParams = readInputFile(inputData.flightComputerFile);


% Apply Monte Carlo Perturbations
[inputData, spaceplane] = applyMonteCarloPerturbations(inputData, perfectSpaceplane, mcParams);



%% FLIGHT COMPUTER INITIALIZATION
flightComputer = initializeFlightComputer(inputData.trajectoryInputFile, inputData.simdt, inputData.sensorMode, inputData.rwyHeading, inputData.fcPhase, ...
    mcParams.initialErrors, mcParams.imu, inputData.state, flightComputerParams, perfectSpaceplane, inputData.lateralAutopilotMode, inputData.longitudinalAutopilotMode, inputData.speedbrakeAutopilotMode);




%% INITIAL CONDITIONS SETUP

npts = numel(inputData.simtStart : inputData.simdt : inputData.simtEnd);
DATA = repmat(simDataPoint(1), npts, 1);
tVector = inputData.simtStart : inputData.simdt : inputData.simtEnd;

for idx = 1:npts
    DATA(idx).t = tVector(idx);
end


% Set the initial state vector to the simulation data structure
DATA(1).state = inputData.state;
DATA(1).state.vX = NaN;
DATA(1).state.vY = NaN;
DATA(1).state.vZ = NaN;
DATA(1).state.aX = NaN;
DATA(1).state.aY = NaN;
DATA(1).state.aZ = NaN;



%% WIND GUST TABLE BUILDING
windGustTimeHistory = buildWindGustTimeHistory(mcParams.gustTimeTable, tVector);
globalWindDirection = mcParams.windDirection * pi/180;
globalWindMultiplier = 1 + mcParams.windMultiplier;



%% MAIN SIMULATION LOOP

for idx = 1:npts-1
    
    
    % Get the sim data point for this time step
    curDataPt = DATA(idx);
    curTime = DATA(idx).t;
    
    
    % Compute dt.
    dt = DATA(idx+1).t - DATA(idx).t;    
    
    
    
    % Current state and control surface commands for this timestep.
    curState = curDataPt.state;
    
    
    
    % Control surface commands.  Control surfaces are not
    % commanded during the first simulation step.  Random noise values for
    % sensors for this time step are fed into the flight computer as well,
    % as they are not stored in the flight computer variable itself due to
    % processing speed concerns.
    if idx > 2
        randomNoise = struct();
        randomNoise.imuAccel = mcParams.imu.accelNoise(idx,:);
        randomNoise.imuGyro = mcParams.imu.gyroNoise(idx,:);
        randomNoise.gpsPos = mcParams.gps.posNoise(idx,:);
        randomNoise.gpsVel = mcParams.gps.velNoise(idx,:);
        randomNoise.rdrAlt = mcParams.rdrAlt.posNoise(idx,:);
        randomNoise.aoa = mcParams.aeroSensor.aoaNoise(idx,1);
        randomNoise.beta = mcParams.aeroSensor.betaNoise(idx,1);

        flightComputer = flightComputerStepUpdate(flightComputer, curState, randomNoise, curTime);
        curCtrlSurf = commandControlSurf(flightComputer);
    else
        curCtrlSurf = zeroCtrlSurf();
    end
    
    
    
    % Stop on ground contact.
    if curDataPt.state.zE > 0
        vspd = -(DATA(idx).state.zE - DATA(idx-1).state.zE) / dt;
        fprintf(['Ground Contact Vertical Speed: ', num2str(vspd, '%6.2f'), '\n']);            
        DATA(idx+1 : end) = [];
        break;
    end
    
    
    
    % We need to calculate the atmospheric properties
    curDataPt.aero.rho = interp1(inputData.standardAtmosphere(:, 1), inputData.standardAtmosphere(:, 4), -curDataPt.state.zE);
    curDataPt.aero.airspeed = norm([curDataPt.state.u, curDataPt.state.v, curDataPt.state.w]);
    curDataPt.aero.ainf = interp1(inputData.standardAtmosphere(:, 1), inputData.standardAtmosphere(:, 5), -curDataPt.state.zE);
    curDataPt.aero.mach = curDataPt.aero.airspeed / curDataPt.aero.ainf;
    curDataPt.aero.Tinf = interp1(inputData.standardAtmosphere(:, 1), inputData.standardAtmosphere(:, 3), -curDataPt.state.zE);
    
    
    % Call the truth model wrapper to propagate the spacecraft to the next
    % state.
    if isempty(windGustTimeHistory)
        windNED = [0 0 0];
        windUVW = [0 0 0];
    else
        [windNED, windUVW] = windModel(curDataPt.state, globalWindDirection, globalWindMultiplier, windGustTimeHistory(idx, :));
    end
    
    outputData = findNextState( dt, ...
                                curState, ...
                                curCtrlSurf, ...
                               -curDataPt.state.zE, ...
                                spaceplane, ...
                                curDataPt.aero.rho, ...
                                curDataPt.aero.mach, ...
                                windUVW, ...
                                inputData.aerodynamicModel, ...
                                inputData.sixDofModel);
    
    
    
    % Record Data
    curDataPt.forceMoment = outputData.forceMoment;
    curDataPt.ctrlSurf = curCtrlSurf;
    curDataPt.wind.windNED = windNED;
    curDataPt.wind.windUVW = windUVW;
    
    % Only pass certain parts through the function for increased speed
    if idx > 2
        curDataPt = setFCParams(curDataPt, flightComputer.dt, flightComputer.phase, flightComputer.Sensors, flightComputer.Navigation, ...
            flightComputer.Guidance, flightComputer.Autopilot, flightComputer.ControlSurfCommand);
    end
    
    DATA(idx) = curDataPt;
    DATA(idx+1).state = outputData.state;
    
    if idx < 2
        DATA(idx+1).state.aX = NaN;
        DATA(idx+1).state.aY = NaN;
        DATA(idx+1).state.aZ = NaN;
    else
        DATA(idx+1).state.aX = (DATA(idx+1).state.vX - DATA(idx).state.vX) ./ dt;
        DATA(idx+1).state.aY = (DATA(idx+1).state.vY - DATA(idx).state.vY) ./ dt;
        DATA(idx+1).state.aZ = (DATA(idx+1).state.vZ - DATA(idx).state.vZ) ./ dt;
    end
    
    
    % Print progress to command window once per simulation second
    if printProgress == true
        if mod(DATA(idx).t, .1) == 0
            fprintf(['Solution Complete through T+', num2str(DATA(idx+1).t, '%6.1f'), ' seconds\n']);
        end
    end
    
    
end
    


%% CLOSEOUT

% Print data to CSV
if saveData == true
    printData(DATA, runId, outputFolder, inputData.simdt, inputData.dataOutputRate);
end


% Make plots
if createPlots == true
    plotData = makePlotData(DATA, inputData.simdt, inputData.dataOutputRate);
    makePlots(inputData, plotData, flightComputer.trajectoryProps)
end


    
end
    