function [inertialNavSoln, integratedNavSoln] = initializeNavSolution(initialErrors, initialState, flightComputerInputParams)
    
    
    % Function to initialize the inertial nav solution and integrated nav
    % solution
    
    inertialNavSoln = struct();
    integratedNavSoln = struct();
    
    
    %% INERTIAL NAV SOLUTION INITIALIZATION
    
    % Attitude Initialization
    rpy0 = [initialState.phi; initialState.theta; initialState.psi];
    Cib_true = Tearth2body(rpy0(1), rpy0(2), rpy0(3));
    Cbi_true = transpose(Cib_true);
    Cbi_error = Tearth2body(-initialErrors.attitude(1) * pi/180, -initialErrors.attitude(2) * pi/180, -initialErrors.attitude(3) * pi/180);
    inertialNavSoln.Cbihat = Cbi_error*Cbi_true;
    inertialNavSoln.rpyhat = transpose(dcm2rpy(inertialNavSoln.Cbihat.'));
    
    % Position Initialization
    inertialNavSoln.rhat = [initialState.xE; initialState.yE; initialState.zE] + initialErrors.position;
    
    
    % Velocity Initialization
    vEtrue = Cbi_true * [initialState.u; initialState.v; initialState.w];
    inertialNavSoln.vhat = vEtrue + initialErrors.velocity;
    
    
    
    %% INTEGRATED NAV SOLUTION INITIALIZATION
    
    % Kalman Filter State
    integratedNavSoln.xhat = zeros(15,1);
    
    
    % Kalman Filter Covariance
    integratedNavSoln.P = zeros(15);
    integratedNavSoln.P(1:3,1:3) = eye(3) .* (flightComputerInputParams.KF.initAttitudeUncertainty * pi/180)^2;
    integratedNavSoln.P(4:6,4:6) = eye(3) .* (flightComputerInputParams.KF.initVelocityUncertainty)^2;
    integratedNavSoln.P(7:9,7:9) = eye(3) .* (flightComputerInputParams.KF.initialPositionUncertainty)^2;
    integratedNavSoln.P(10:12,10:12) = eye(3) .* (flightComputerInputParams.KF.initialAccelBiasUncertainty)^2;
    integratedNavSoln.P(13:15,13:15) = eye(3) .* (flightComputerInputParams.KF.initialGyroBiasUncertainty)^2;
    
    integratedNavSoln.stdev = nan(15,1);
    for idx = 1:15
        integratedNavSoln.stdev(idx) = sqrt(integratedNavSoln.P(idx,idx));
    end
    
    integratedNavSoln.Q = eye(15);
    integratedNavSoln.Q(1:3,1:3) = eye(3) .* flightComputerInputParams.KF.Q11;
    integratedNavSoln.Q(4:6,4:6) = eye(3) .* flightComputerInputParams.KF.Q22;
    integratedNavSoln.Q(7:9,7:9) = eye(3) .* flightComputerInputParams.KF.Q33;
    integratedNavSoln.Q(10:12,10:12) = eye(3) .* flightComputerInputParams.KF.Q44;
    integratedNavSoln.Q(13:15,13:15) = eye(3) .* flightComputerInputParams.KF.Q55;
    
    integratedNavSoln.R = eye(3);
    integratedNavSoln.R(1:3,1:3) = eye(3) .* (flightComputerInputParams.gps.posMeasurementSigma)^2;
    
    integratedNavSoln.H = zeros(3, 15);
    integratedNavSoln.H(1:3, 7:9) = -eye(3);
    
    integratedNavSoln.biasEstimates.accel = zeros(3,1);
    integratedNavSoln.biasEstimates.gyro = zeros(3,1);
    
    integratedNavSoln.updatePeriod = flightComputerInputParams.KF.updatePeriod;
    integratedNavSoln.nextUpdateTime = flightComputerInputParams.KF.updatePeriod;
    
    integratedNavSoln.gpsMeasurement.t = 0;
    integratedNavSoln.gpsMeasurement.pos = nan(3,1);
    integratedNavSoln.gpsMeasurement.vel = nan(3,1);
    
    
end