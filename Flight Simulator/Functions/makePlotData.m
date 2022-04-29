function plotData = makePlotData(Data, simdt, outputRate)

    
    % Preallocation of plot data structure
    nptsTotal = numel(Data);
    
    interval = (1/simdt) / outputRate;
    nptsToPrint = numel(0 : 1/outputRate : Data(end).t);
    
    plotData = simDataPoint(nptsToPrint);
    
    idxWrite = 1;
    
    for idx = 1 : interval : nptsTotal
        
        plotData.t(idxWrite) = Data(idx).t;

        plotData.rho(idxWrite) = Data(idx).aero.rho;
        plotData.mach(idxWrite) = Data(idx).aero.mach;
        plotData.airspeed(idxWrite) = Data(idx).aero.airspeed;
        plotData.Tinf(idxWrite) = Data(idx).aero.Tinf;


        % Control Surfaces
        plotData.ctrlSurf.gear(idxWrite) = Data(idx).ctrlSurf.gear;
        plotData.ctrlSurf.spdbk(idxWrite) = Data(idx).ctrlSurf.spdbk;
        plotData.ctrlSurf.elevator(idxWrite) = Data(idx).ctrlSurf.elevator;
        plotData.ctrlSurf.aileron(idxWrite) = Data(idx).ctrlSurf.aileron;
        plotData.ctrlSurf.rudder(idxWrite) = Data(idx).ctrlSurf.rudder;
        plotData.ctrlSurf.trim(idxWrite) = Data(idx).ctrlSurf.trim;


        % True state and derivative of true state
        plotData.state.u(idxWrite) = Data(idx).state.u;
        plotData.state.v(idxWrite) = Data(idx).state.v;
        plotData.state.w(idxWrite) = Data(idx).state.w;
        plotData.state.p(idxWrite) = Data(idx).state.p;
        plotData.state.q(idxWrite) = Data(idx).state.q;
        plotData.state.r(idxWrite) = Data(idx).state.r;
        plotData.state.psi(idxWrite) = Data(idx).state.psi;
        plotData.state.theta(idxWrite) = Data(idx).state.theta;
        plotData.state.phi(idxWrite) = Data(idx).state.phi;
        plotData.state.xE(idxWrite) = Data(idx).state.xE;
        plotData.state.yE(idxWrite) = Data(idx).state.yE;
        plotData.state.zE(idxWrite) = Data(idx).state.zE;
        plotData.state.vX(idxWrite) = Data(idx).state.vX;
        plotData.state.vY(idxWrite) = Data(idx).state.vY;
        plotData.state.vZ(idxWrite) = Data(idx).state.vZ;
        plotData.state.aX(idxWrite) = Data(idx).state.aX;
        plotData.state.aY(idxWrite) = Data(idx).state.aY;
        plotData.state.aZ(idxWrite) = Data(idx).state.aZ;
        plotData.state.vE(idxWrite) = norm([Data(idx).state.vX, Data(idx).state.vY, Data(idx).state.vZ]);


        % Forces and moments
        plotData.forceMoment.netForceVecInB(idxWrite, :) = Data(idx).forceMoment.netForceVecInB;
        plotData.forceMoment.liftVecInB(idxWrite, :) = Data(idx).forceMoment.liftVecInB;
        plotData.forceMoment.dragVecInB(idxWrite, :) = Data(idx).forceMoment.dragVecInB;
        plotData.forceMoment.weightVecInB(idxWrite, :) = Data(idx).forceMoment.weightVecInB;
        plotData.forceMoment.momentVecInB(idxWrite, :) = Data(idx).forceMoment.momentVecInB;


        % Flight Computer
        plotData.FC.dt(idxWrite) = Data(idx).FC.dt;
        plotData.FC.phase(idxWrite) = Data(idx).FC.phase;
        plotData.FC.Sensors.truthSensor.xE(idxWrite) = Data(idx).FC.Sensors.truthSensor.xE;
        plotData.FC.Sensors.truthSensor.yE(idxWrite) = Data(idx).FC.Sensors.truthSensor.yE;
        plotData.FC.Sensors.truthSensor.zE(idxWrite) = Data(idx).FC.Sensors.truthSensor.zE;
        plotData.FC.Sensors.truthSensor.xEdot(idxWrite) = Data(idx).FC.Sensors.truthSensor.xEdot;
        plotData.FC.Sensors.truthSensor.yEdot(idxWrite) = Data(idx).FC.Sensors.truthSensor.yEdot;
        plotData.FC.Sensors.truthSensor.zEdot(idxWrite) = Data(idx).FC.Sensors.truthSensor.zEdot;
        plotData.FC.Sensors.truthSensor.u(idxWrite) = Data(idx).FC.Sensors.truthSensor.u;
        plotData.FC.Sensors.truthSensor.v(idxWrite) = Data(idx).FC.Sensors.truthSensor.v;
        plotData.FC.Sensors.truthSensor.w(idxWrite) = Data(idx).FC.Sensors.truthSensor.w;
        plotData.FC.Sensors.truthSensor.phi(idxWrite) = Data(idx).FC.Sensors.truthSensor.phi;
        plotData.FC.Sensors.truthSensor.theta(idxWrite) = Data(idx).FC.Sensors.truthSensor.theta;
        plotData.FC.Sensors.truthSensor.psi(idxWrite) = Data(idx).FC.Sensors.truthSensor.psi;
        plotData.FC.Sensors.truthSensor.p(idxWrite) = Data(idx).FC.Sensors.truthSensor.p;
        plotData.FC.Sensors.truthSensor.q(idxWrite) = Data(idx).FC.Sensors.truthSensor.q;
        plotData.FC.Sensors.truthSensor.r(idxWrite) = Data(idx).FC.Sensors.truthSensor.r;
        plotData.FC.Sensors.truthSensor.aoa(idxWrite) = Data(idx).FC.Sensors.truthSensor.aoa;
        plotData.FC.Sensors.truthSensor.beta(idxWrite) = Data(idx).FC.Sensors.truthSensor.beta;
        
        plotData.FC.Navigation.Filters.InertialNavSoln.rhat(idxWrite, :) = Data(idx).FC.Navigation.Filters.InertialNavSoln.rhat;
        plotData.FC.Navigation.Filters.InertialNavSoln.vhat(idxWrite, :) = Data(idx).FC.Navigation.Filters.InertialNavSoln.vhat;
        plotData.FC.Navigation.Filters.InertialNavSoln.rpyhat(idxWrite, :) = Data(idx).FC.Navigation.Filters.InertialNavSoln.rpyhat;
        plotData.FC.Navigation.Filters.IntegratedNavSoln.xhat(idxWrite, :) = Data(idx).FC.Navigation.Filters.IntegratedNavSoln.xhat;
        plotData.FC.Navigation.Filters.IntegratedNavSoln.stdev(idxWrite, :) = Data(idx).FC.Navigation.Filters.IntegratedNavSoln.stdev;
        plotData.FC.Navigation.Filters.IntegratedNavSoln.biasEstimates.accel(idxWrite, :) = Data(idx).FC.Navigation.Filters.IntegratedNavSoln.biasEstimates.accel;
        plotData.FC.Navigation.Filters.IntegratedNavSoln.biasEstimates.gyro(idxWrite, :) = Data(idx).FC.Navigation.Filters.IntegratedNavSoln.biasEstimates.gyro;
        plotData.FC.Navigation.Filters.IntegratedNavSoln.updatePeriod(idxWrite, :) = Data(idx).FC.Navigation.Filters.IntegratedNavSoln.updatePeriod;
        plotData.FC.Navigation.Filters.IntegratedNavSoln.nextUpdateTime(idxWrite, :) = Data(idx).FC.Navigation.Filters.IntegratedNavSoln.nextUpdateTime;

        
        plotData.FC.Navigation.ControlVariables.crossTrackError(idxWrite) = Data(idx).FC.Navigation.ControlVariables.crossTrackError;
        plotData.FC.Navigation.ControlVariables.crossTrackErrorRate(idxWrite) = Data(idx).FC.Navigation.ControlVariables.crossTrackErrorRate;
        plotData.FC.Navigation.ControlVariables.LonVars.alpha(idxWrite) = Data(idx).FC.Navigation.ControlVariables.LonVars.alpha;
        plotData.FC.Navigation.ControlVariables.LonVars.q(idxWrite) = Data(idx).FC.Navigation.ControlVariables.LonVars.q;
        plotData.FC.Navigation.ControlVariables.LonVars.theta(idxWrite) = Data(idx).FC.Navigation.ControlVariables.LonVars.theta;
        plotData.FC.Navigation.ControlVariables.LatVars.beta(idxWrite) = Data(idx).FC.Navigation.ControlVariables.LatVars.beta;
        plotData.FC.Navigation.ControlVariables.LatVars.p(idxWrite) = Data(idx).FC.Navigation.ControlVariables.LatVars.p;
        plotData.FC.Navigation.ControlVariables.LatVars.r(idxWrite) = Data(idx).FC.Navigation.ControlVariables.LatVars.r;
        plotData.FC.Navigation.ControlVariables.LatVars.phi(idxWrite) = Data(idx).FC.Navigation.ControlVariables.LatVars.phi;
        plotData.FC.Navigation.ControlVariables.airspeed(idxWrite) = Data(idx).FC.Navigation.ControlVariables.airspeed;
        plotData.FC.Navigation.ControlVariables.posE(idxWrite, :) = Data(idx).FC.Navigation.ControlVariables.posE;
        plotData.FC.Navigation.ControlVariables.velE(idxWrite, :) = Data(idx).FC.Navigation.ControlVariables.velE;
        plotData.FC.Navigation.ControlVariables.posTaem(idxWrite, :) = Data(idx).FC.Navigation.ControlVariables.posTaem;
        plotData.FC.Navigation.ControlVariables.velTaem(idxWrite, :) = Data(idx).FC.Navigation.ControlVariables.velTaem;
        plotData.FC.Navigation.ControlVariables.rHac(idxWrite) = Data(idx).FC.Navigation.ControlVariables.rHac;
        plotData.FC.Navigation.ControlVariables.psiHac(idxWrite) = Data(idx).FC.Navigation.ControlVariables.psiHac;
        plotData.FC.Navigation.ControlVariables.rdotHac(idxWrite) = Data(idx).FC.Navigation.ControlVariables.rdotHac;
        plotData.FC.Guidance.Gains.Vnav.KP(idxWrite) = Data(idx).FC.Guidance.Gains.Vnav.KP;
        plotData.FC.Guidance.Gains.Vnav.KI(idxWrite) = Data(idx).FC.Guidance.Gains.Vnav.KI;
        plotData.FC.Guidance.Gains.Vnav.KD(idxWrite) = Data(idx).FC.Guidance.Gains.Vnav.KD;
        plotData.FC.Guidance.Gains.Lnav.KP(idxWrite) = Data(idx).FC.Guidance.Gains.Lnav.KP;
        plotData.FC.Guidance.Gains.Lnav.KI(idxWrite) = Data(idx).FC.Guidance.Gains.Lnav.KI;
        plotData.FC.Guidance.Gains.Lnav.KD(idxWrite) = Data(idx).FC.Guidance.Gains.Lnav.KD;
        plotData.FC.Guidance.Commands.altitude(idxWrite) = Data(idx).FC.Guidance.Commands.altitude;
        plotData.FC.Guidance.Commands.theta(idxWrite) = Data(idx).FC.Guidance.Commands.theta;
        plotData.FC.Guidance.Commands.phi(idxWrite) = Data(idx).FC.Guidance.Commands.phi;
        plotData.FC.Guidance.Commands.airspeed(idxWrite) = Data(idx).FC.Guidance.Commands.airspeed;
        plotData.FC.Guidance.Commands.spdbk(idxWrite) = Data(idx).FC.Guidance.Commands.spdbk;
        plotData.FC.Guidance.Commands.spdbkControllerLevel(idxWrite) = Data(idx).FC.Guidance.Commands.spdbkControllerLevel;
        plotData.FC.Guidance.Integrators.altError(idxWrite) = Data(idx).FC.Guidance.Integrators.altError;
        plotData.FC.Guidance.Integrators.crossTrackError(idxWrite) = Data(idx).FC.Guidance.Integrators.crossTrackError;
        plotData.FC.Autopilot.Gains.Vnav.K(idxWrite, :) = Data(idx).FC.Autopilot.Gains.Vnav.K;
        plotData.FC.Autopilot.Gains.Vnav.H(idxWrite) = Data(idx).FC.Autopilot.Gains.Vnav.H;
        plotData.FC.Autopilot.Gains.Lnav.K(idxWrite, :) = Data(idx).FC.Autopilot.Gains.Lnav.K;
        plotData.FC.Autopilot.Gains.Lnav.H(idxWrite) = Data(idx).FC.Autopilot.Gains.Lnav.H;
        plotData.FC.Autopilot.Integrators.thetaError(idxWrite) = Data(idx).FC.Autopilot.Integrators.thetaError;
        plotData.FC.Autopilot.Integrators.phiError(idxWrite) = Data(idx).FC.Autopilot.Integrators.phiError;
        plotData.FC.ControlSurfCommand.gear(idxWrite) = Data(idx).FC.ControlSurfCommand.gear;
        plotData.FC.ControlSurfCommand.trim(idxWrite) = Data(idx).FC.ControlSurfCommand.trim;
        plotData.FC.ControlSurfCommand.elevator(idxWrite) = Data(idx).FC.ControlSurfCommand.elevator;
        plotData.FC.ControlSurfCommand.aileron(idxWrite) = Data(idx).FC.ControlSurfCommand.aileron;
        plotData.FC.ControlSurfCommand.rudder(idxWrite) = Data(idx).FC.ControlSurfCommand.rudder;
        plotData.FC.ControlSurfCommand.spdbk(idxWrite) = Data(idx).FC.ControlSurfCommand.spdbk;

        idxWrite = idxWrite + 1;
        
    end
    
end