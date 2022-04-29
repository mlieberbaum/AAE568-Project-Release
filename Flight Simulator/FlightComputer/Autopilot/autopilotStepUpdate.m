function FC = autopilotStepUpdate(FC)


    %% TRIM AIRFRAME
    
    % If using truth mode, use the truth state to trim.  Otherwise, use the
    % state estimate
    if FC.useLateralMPC || FC.useLongitudinalMPC || FC.useSpeedbrakeMPC
        if FC.useTruth == 1
            aspdTruth = norm([FC.Sensors.truthSensor.u, FC.Sensors.truthSensor.v, FC.Sensors.truthSensor.w]);
            altTruth = max(-FC.Sensors.truthSensor.zE, 0);
            [rho, mach] = standardAtmosphereLookup(altTruth, aspdTruth);
            trimPoint = trimAirframe(FC.spaceplane, aspdTruth, altTruth, rho, mach);
            aspdSpdbkCtrl = aspdTruth;
            rhoSpdbkCtrl = rho;
        else
            aspdEst = norm([FC.Sensors.truthSensor.u, FC.Sensors.truthSensor.v, FC.Sensors.truthSensor.w]);
            altitudeEst = max(-FC.Navigation.Filters.InertialNavSoln.rhat(3), 0);
            [rhoEst, machEst] = standardAtmosphereLookup(altitudeEst, aspdEst);
            trimPoint = trimAirframe(FC.spaceplane, aspdEst, altitudeEst, rhoEst, machEst);
            aspdSpdbkCtrl = aspdEst;
            rhoSpdbkCtrl = rhoEst;
        end
    end

    
    %% LONGITUDINAL X-XE CONTROL LAW
    
    if FC.useLongitudinalMPC == 0
        
        % LQR with integrator
        xV = [FC.Navigation.ControlVariables.LonVars.alpha
              FC.Navigation.ControlVariables.LonVars.q
              FC.Navigation.ControlVariables.LonVars.theta - FC.Guidance.Commands.theta];

        thetaErr = -xV(3);

        FC.ControlSurfCommand.elevator = -FC.Autopilot.Gains.Vnav.K * xV - FC.Autopilot.Integrators.thetaError * FC.Autopilot.Gains.Vnav.H;
        FC.Autopilot.Integrators.thetaError = FC.Autopilot.Integrators.thetaError + thetaErr*FC.dt;
        
    else
        
        Alon = trimPoint.lon.Actl(1:3,1:3);
        Blon = trimPoint.lon.Bctl(1:3);
        M = [Alon, Blon
             zeros(1,4)];
        Md = expm(M * FC.dt);
        
        Ad = Md(1:3,1:3);
        Bd = Md(1:3,4);
        Cd = [0 0 1];
        
        Atilde = [Ad,    zeros(3,1)
                  Cd*Ad  1];
        Btilde = [Bd
                  Cd*Bd];
        Ctilde = [0 0 0 1];
        
        N = 100;
        Q = eye(4);
        R = 1;

        xTgt = [0;0;0;FC.Guidance.Commands.theta];
        duCmd = solveAugmentedMpc(Atilde, Btilde, Ctilde, N, Q, R, FC.Autopilot.MPC.Longitudinal.xTilde, FC.Autopilot.MPC.Longitudinal.curU, ...
            1, 1, xTgt, FC.Autopilot.MPC.pitchLimit, FC.Autopilot.MPC.maxCtrlSurfDeflectionDelta);

        FC.Autopilot.MPC.Longitudinal.curU = FC.Autopilot.MPC.Longitudinal.curU + duCmd;
        FC.ControlSurfCommand.elevator = FC.Autopilot.MPC.Longitudinal.curU;
        xCur = [FC.Navigation.ControlVariables.LonVars.alpha
                FC.Navigation.ControlVariables.LonVars.q
                FC.Navigation.ControlVariables.LonVars.theta];
        yCur = FC.Navigation.ControlVariables.LonVars.theta;

        FC.Autopilot.MPC.Longitudinal.xTilde = [xCur - FC.Autopilot.MPC.Longitudinal.xPrev; yCur];
        FC.Autopilot.MPC.Longitudinal.xPrev = xCur;
        
    end
    
    
    
    %% LATERAL
    
    
    if FC.useLateralMPC == 0
        
        % LQR with integrator
        xL = [FC.Navigation.ControlVariables.LatVars.beta
              FC.Navigation.ControlVariables.LatVars.p
              FC.Navigation.ControlVariables.LatVars.r
              FC.Navigation.ControlVariables.LatVars.phi - FC.Guidance.Commands.phi];

        phiError = -xL(4);

        FC.ControlSurfCommand.aileron = -FC.Autopilot.Gains.Lnav.K * xL - FC.Autopilot.Integrators.phiError * FC.Autopilot.Gains.Lnav.H;
        FC.Autopilot.Integrators.phiError = FC.Autopilot.Integrators.phiError + phiError*FC.dt;
        
    else
        
        Alat = trimPoint.lat.Actl;
        Blat = trimPoint.lat.Bctl;
        M = [Alat, Blat
             zeros(2,6)];
        Md = expm(M * FC.Autopilot.updatePeriod);
        
        Ad = Md(1:4,1:4);
        Bd = Md(1:4,5:6);
        Cd = [1 0 0 0
              0 0 0 1];
        
        Atilde = [Ad,    zeros(4,2)
                  Cd*Ad  eye(2)];
        Btilde = [Bd
                  Cd*Bd];
        Ctilde = [zeros(2,4), eye(2)];
        
        N = 50;
        Q = 3*eye(4);
        R = eye(2);

        xTgt = [0;0;0;0;0;FC.Guidance.Commands.phi];
        duCmd = solveAugmentedMpcTwoInput(Atilde, Btilde, Ctilde, N, Q, R, FC.Autopilot.MPC.Lateral.xTilde, FC.Autopilot.MPC.Lateral.curU, 1, xTgt, ...
            [FC.Autopilot.MPC.sideslipLimit; FC.Autopilot.MPC.rollLimit], FC.Autopilot.MPC.maxCtrlSurfDeflectionDelta);

        FC.Autopilot.MPC.Lateral.curU = FC.Autopilot.MPC.Lateral.curU + duCmd;
        
        FC.ControlSurfCommand.aileron = FC.Autopilot.MPC.Lateral.curU(1);
        FC.ControlSurfCommand.rudder = FC.Autopilot.MPC.Lateral.curU(2);
        
        xCur = [FC.Navigation.ControlVariables.LatVars.beta
                FC.Navigation.ControlVariables.LatVars.p
                FC.Navigation.ControlVariables.LatVars.r
                FC.Navigation.ControlVariables.LatVars.phi];
        
        yCur = [FC.Navigation.ControlVariables.LatVars.beta;
                FC.Navigation.ControlVariables.LatVars.phi];

        FC.Autopilot.MPC.Lateral.xTilde = [xCur - FC.Autopilot.MPC.Lateral.xPrev; yCur];
        FC.Autopilot.MPC.Lateral.xPrev = xCur;
        
    end
    
    
    
    
    %% SPEEDBRAKE
    
    
    if FC.useSpeedbrakeMPC == 0
        
        % Starting at pre-flare, speedbrake command is zero
        if strcmp(FC.phase, 'PRE_FLARE') || strcmp(FC.phase, 'SHORT_FINAL') || strcmp(FC.phase, 'FLARE')
            FC.ControlSurfCommand.spdbk = 0;
            return;
        end
    
        switch FC.phase
            case 'TAEM'
                spdbkNominal = FC.trajectoryProps.taem.spdbkNominal;
            case 'HAC'
                spdbkNominal = FC.trajectoryProps.hac.spdbkNominal;
            case 'LONG_FINAL'
                spdbkNominal = FC.trajectoryProps.longFinal.spdbkNominal;
            otherwise
                spdbkNominal = 0;
        end
    
        FC.ControlSurfCommand.spdbk = spdbkNominal + (-0.05 * (FC.Guidance.Commands.airspeed - FC.Navigation.ControlVariables.airspeed));
        
    else

        % Trim based on qinf and space plane
        Asb = 0;
        Bsb = (-(1/2) * rhoSpdbkCtrl * aspdSpdbkCtrl^2 * FC.spaceplane.speedbrake.F) / FC.spaceplane.mass;

        M = [Asb, Bsb
             zeros(1,2)];
        Md = expm(M * FC.Autopilot.updatePeriod);

        Ad = Md(1,1);
        Bd = Md(1,2);
        Cd = 1;

        Atilde = [Ad,    zeros(1)
                  Cd*Ad  1];
        Btilde = [Bd
                  Cd*Bd];
        Ctilde = [0 1];

        N = 50;
        Q = .001;
        R = 1;

        xTgt = [0; FC.Guidance.Commands.airspeed];
        duCmd = solveAugmentedMpc(Atilde, Btilde, Ctilde, N, Q, R, FC.Autopilot.MPC.Speedbrake.xTilde, FC.Autopilot.MPC.Speedbrake.curU, 1, 0, ...
            xTgt, 1e6, FC.Autopilot.MPC.maxCtrlSurfDeflectionDelta);

        FC.Autopilot.MPC.Speedbrake.curU = FC.Autopilot.MPC.Speedbrake.curU + duCmd;

        FC.ControlSurfCommand.spdbk = FC.Autopilot.MPC.Speedbrake.curU;

        xCur = aspdSpdbkCtrl;
        yCur = aspdSpdbkCtrl;

        FC.Autopilot.MPC.Speedbrake.xTilde = [xCur - FC.Autopilot.MPC.Speedbrake.xPrev; yCur];
        FC.Autopilot.MPC.Speedbrake.xPrev = xCur;
        
    end
    
    
end