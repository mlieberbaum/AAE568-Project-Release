function [inertialNavSolution, insNavSolution] = navSolnStepUpdate(inertialNavSolution, insNavSolution, spfrcMsmt_body, gyroMsmt, gpsMsmt, rdrAlt, curTime, dt)

    % Propagate the inertial solution forward in time
    
    % Attitude
    Ombi = skewSymmetric((gyroMsmt - insNavSolution.biasEstimates.gyro));
    CbihatNew = repairMatrix(inertialNavSolution.Cbihat * expm(Ombi * dt)); % Groves 5.9
    
    
    % Velocity & Position
    spfrcMsmt_i = inertialNavSolution.Cbihat * (spfrcMsmt_body - insNavSolution.biasEstimates.accel); % Groves 5.15
    gMeas_i = [0; 0; gravityModel(-inertialNavSolution.rhat(3))];
    accelMsmt_i = spfrcMsmt_i + gMeas_i; % Groves 5.18
    
    vhatCur = inertialNavSolution.vhat;
    vhatNew = inertialNavSolution.vhat + accelMsmt_i .* dt;
    inertialNavSolution.rhat = inertialNavSolution.rhat + 0.5 * (vhatCur + vhatNew) * dt; % Groves 5.23a
    
    
    % Set new variables
    inertialNavSolution.Cbihat = CbihatNew;
    inertialNavSolution.rpyhat = dcm2rpy(inertialNavSolution.Cbihat.').';
    inertialNavSolution.vhat = vhatNew;
    
    
    
    
    
    %% ERROR STATE EKF
    
    if curTime >= insNavSolution.nextUpdateTime
        
        I3 = eye(3);
        Z3 = zeros(3);
        
        dtKF = insNavSolution.updatePeriod;
        
        % Compute phi matrix
        Cbih = inertialNavSolution.Cbihat;
        F21 = -skewSymmetric(Cbih * spfrcMsmt_body);   % Groves 14.36
        F23 = zeros(3);                                % Groves 14.36
        ghat = 398600439968871 / ((-inertialNavSolution.rhat(3) + 6371010.0)^2);
        F23(3,3) = -2*ghat/6371010;
        
        F = [Z3           Z3         Z3          Z3           Cbih
             F21          Z3         F23         Cbih         Z3
             Z3           I3         Z3          Z3           Z3
             Z3           Z3         Z3          Z3           Z3
             Z3           Z3         Z3          Z3           Z3];
        
        Phi = expm(F*dtKF);
        
        
        
        % Propagate state (just zeros)
        xhat_k = [zeros(9,1); insNavSolution.biasEstimates.accel; insNavSolution.biasEstimates.gyro];
        xhat_k_1 = Phi * xhat_k;
        
        
        % Propagate Covariance
        P_k_1 = insNavSolution.P;
        P_propagated = PropCovar(P_k_1, F, insNavSolution.Q, dtKF);
        
        
        % Kalman Gain
        if rdrAlt.state == 1
            insNavSolution.R(3,3) = 1;
        end
        K = P_propagated * insNavSolution.H' * inv(insNavSolution.H * P_propagated * insNavSolution.H' + insNavSolution.R);
        
        
        % GPS Measurement (Groves 14.101)
        z = gpsMsmt.posMsmt;
        if rdrAlt.state == 1
            z(3) = rdrAlt.msmt;
        end
        delz = z - inertialNavSolution.rhat;
        
        
        xhat_k = xhat_k_1 + K * delz;
        insNavSolution.P = (eye(15) - K * insNavSolution.H) * P_propagated;
        
        
        % Correct Attitude
        inertialNavSolution.Cbihat = repairMatrix((eye(3) - skewSymmetric(xhat_k(1:3))) * inertialNavSolution.Cbihat);
        inertialNavSolution.rpyhat = dcm2rpy(inertialNavSolution.Cbihat.')';
        
        
        % Correct Position and Velocity
        inertialNavSolution.vhat = inertialNavSolution.vhat - xhat_k(4:6);
        inertialNavSolution.rhat = inertialNavSolution.rhat - xhat_k(7:9);
        
        
        % Update bias estimate
        insNavSolution.biasEstimates.accel = xhat_k(10:12);
        insNavSolution.biasEstimates.gyro = xhat_k(13:15);
        
        
        % Compute standard deviations
        for idx2 = 1:15
            insNavSolution.stdev(idx2) = sqrt(insNavSolution.P(idx2, idx2));
        end
        
        insNavSolution.gpsMeasurement.t = curTime;
        insNavSolution.gpsMeasurement.pos = gpsMsmt.posMsmt;
        insNavSolution.gpsMeasurement.vel = gpsMsmt.velMsmt;
        
        insNavSolution.nextUpdateTime = insNavSolution.nextUpdateTime + insNavSolution.updatePeriod;
        
    end
    
    
    
    
end