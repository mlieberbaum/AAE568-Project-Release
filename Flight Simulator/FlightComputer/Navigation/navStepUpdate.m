function FC = navStepUpdate(FC, curTime)

    
    %% TRUTH SENSOR UPDATES
        
    posE = [FC.Sensors.truthSensor.xE
            FC.Sensors.truthSensor.yE
            FC.Sensors.truthSensor.zE];

    velE = [FC.Sensors.truthSensor.xEdot
            FC.Sensors.truthSensor.yEdot
            FC.Sensors.truthSensor.zEdot];

    yprE = [FC.Sensors.truthSensor.psi
            FC.Sensors.truthSensor.theta
            FC.Sensors.truthSensor.phi];

    omegaE = [FC.Sensors.truthSensor.p
              FC.Sensors.truthSensor.q
              FC.Sensors.truthSensor.r];

    uvw = [FC.Sensors.truthSensor.u
           FC.Sensors.truthSensor.v
           FC.Sensors.truthSensor.w];
        
    
    
    %% NAV SOLUTION UPDATE
    [FC.Navigation.Filters.InertialNavSoln, FC.Navigation.Filters.IntegratedNavSoln] = navSolnStepUpdate(...
        FC.Navigation.Filters.InertialNavSoln, FC.Navigation.Filters.IntegratedNavSoln, FC.Sensors.IMU.accel.specificForceMsmt, ...
        FC.Sensors.IMU.gyro.omegaMsmt, FC.Sensors.GPS, FC.Sensors.rdrAlt, curTime, FC.dt);
    
    
    
    %% AUTOPILOT CONTROL VARIABLES
    
    if FC.useTruth == 1
        omegaE_ctl = omegaE;
        yprE_ctl = yprE;
        posE_ctl = posE;
        velE_ctl = velE;
        alpha = FC.Sensors.truthSensor.aoa;
        beta = FC.Sensors.truthSensor.beta;
    else
        if any(isnan(FC.Sensors.IMU.filteredMeas.gyro))
            omegaE_ctl = FC.Sensors.IMU.gyro.omegaMsmt;
        else
            omegaE_ctl = mean(FC.Sensors.IMU.filteredMeas.gyro, 1)';
        end
        yprE_ctl = FC.Navigation.Filters.InertialNavSoln.rpyhat(end:-1:1);
        posE_ctl = FC.Navigation.Filters.InertialNavSoln.rhat;
        velE_ctl = FC.Navigation.Filters.InertialNavSoln.vhat;
        
        if any(isnan(FC.Sensors.aoaSensor))
            alpha = FC.Sensors.truthSensor.aoa;
            beta = FC.Sensors.truthSensor.beta;
        else
            alpha = mean(FC.Sensors.aoaSensor);
            beta = mean(FC.Sensors.betaSensor);
        end
    end
    
    FC.Navigation.ControlVariables.LonVars.alpha = alpha;
    FC.Navigation.ControlVariables.LonVars.q = omegaE_ctl(2);
    FC.Navigation.ControlVariables.LonVars.theta = yprE_ctl(2);
    FC.Navigation.ControlVariables.LonVars.gamma = -atan(velE_ctl(3) / norm([velE_ctl(1:2); 0]));
    
    FC.Navigation.ControlVariables.LatVars.beta = beta;
    FC.Navigation.ControlVariables.LatVars.p = omegaE_ctl(1);
    FC.Navigation.ControlVariables.LatVars.r = omegaE_ctl(3);
    FC.Navigation.ControlVariables.LatVars.phi = yprE_ctl(3);
    
    FC.Navigation.ControlVariables.airspeed = norm(uvw);
    
    
    
    %% NAV STATE COORDINATE TRANSFORMATIONS
    
    
    % ---------------------------------------------------------------------
    % Position & Velocity estimate in the Earth frame
    FC.Navigation.ControlVariables.posEprev = FC.Navigation.ControlVariables.posE;
    FC.Navigation.ControlVariables.velEprev = FC.Navigation.ControlVariables.velE;
    FC.Navigation.ControlVariables.posE = posE_ctl;
    FC.Navigation.ControlVariables.velE = velE_ctl;
    
    
    
    % ---------------------------------------------------------------------
    % Position & Velocity estimate in the TAEM frame 
    Otaem_nwu = FC.trajectoryProps.waypoints.wp2;
    Otaem_ned = [Otaem_nwu(1); -Otaem_nwu(2); Otaem_nwu(3)];
    posTaem = posE_ctl - Otaem_ned;
    th = 2*pi - FC.trajectoryProps.taem.thFromPlusxRwy;
    T_e2taem = [ cos(th)  sin(th)  0
                -sin(th)  cos(th)  0
                 0        0        1];
    FC.Navigation.ControlVariables.posTaem = T_e2taem * posTaem;
    FC.Navigation.ControlVariables.velTaem = T_e2taem * velE_ctl;
    
    
    
    % ---------------------------------------------------------------------
    % HAC current radius and psi angle
    Ohac_nwu = FC.trajectoryProps.hac.origin.';
    posNWU = [posE_ctl(1); -posE_ctl(2); -posE_ctl(3)];
    posHacFrame = posNWU - Ohac_nwu;
    velNWU = [velE_ctl(1); -velE_ctl(2); -velE_ctl(3)];
    FC.Navigation.ControlVariables.rHac = norm([posHacFrame(1:2); 0]);
    FC.Navigation.ControlVariables.psiHac = wrapTo2Pi(atan2(posHacFrame(2), posHacFrame(1)) + 3*pi/2);
    
    % Flight path angle and cross track rate for HAC
    r1 = FC.trajectoryProps.hac.r1;
    r2 = FC.trajectoryProps.hac.r2;
    psi = FC.Navigation.ControlVariables.psiHac;
    rcmd = r1 + r2*(psi^2);
    dydpsi = 2*r2*psi*cos(psi) - (r1 + r2*psi*psi)*sin(psi);
    dxdpsi = -2*r2*psi*sin(psi) - (r1 + r2*psi^2)*cos(psi);
    gammaHorizNominal = atan2(-dydpsi, -dxdpsi);
    gammaHoriz = atan2(velNWU(2), velNWU(1));
    FC.Navigation.ControlVariables.rdotHac = norm([velE_ctl(1:2); 0]) * sin(gammaHorizNominal - gammaHoriz);
    
    
    
    % ---------------------------------------------------------------------
    % Cross Track Error and cross track error rate
    if strcmp(FC.phase, 'TAEM')
        y = FC.Navigation.ControlVariables.posTaem(2);
        ydot = FC.Navigation.ControlVariables.velTaem(2);
    elseif strcmp(FC.phase, 'HAC')
        y = FC.Navigation.ControlVariables.rHac - rcmd;
        ydot = FC.Navigation.ControlVariables.rdotHac;
    else
        y = FC.Navigation.ControlVariables.posE(2);
        ydot = FC.Navigation.ControlVariables.velE(2);
    end
    
    FC.Navigation.ControlVariables.crossTrackError = -y;
    FC.Navigation.ControlVariables.crossTrackErrorRate = -ydot;
    
    
    % ---------------------------------------------------------------------
    % Vertical Error Rate
    
    % Performed in guidanceStepUpdate
    
    
    
    
end