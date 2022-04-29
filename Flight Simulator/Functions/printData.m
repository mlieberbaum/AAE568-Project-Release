function printData(simData, runId, outputFolder, simdt, outputRate)


    % Determine the size of the matrix
    nptsTotal = numel(simData);
    
    interval = (1/simdt) / outputRate;
    nptsToPrint = numel(0 : 1/outputRate : simData(end).t);
    
    M = nan(nptsToPrint, 196);
    
    idxWrite = 1;
    
    for idx = 1 : interval : nptsTotal
            
            M(idxWrite, 1) = simData(idx).t;
            M(idxWrite, 2) = simData(idx).aero.rho;
            M(idxWrite, 3) = simData(idx).aero.mach;
            M(idxWrite, 4) = simData(idx).aero.airspeed;
            M(idxWrite, 5) = simData(idx).aero.Tinf;
            
            M(idxWrite, 6) = simData(idx).ctrlSurf.gear;
            M(idxWrite, 7) = simData(idx).ctrlSurf.spdbk;
            M(idxWrite, 8) = simData(idx).ctrlSurf.elevator;
            M(idxWrite, 9) = simData(idx).ctrlSurf.aileron;
            M(idxWrite, 10) = simData(idx).ctrlSurf.rudder;
            M(idxWrite, 11) = simData(idx).ctrlSurf.trim;
            
            M(idxWrite, 12) = simData(idx).state.u;
            M(idxWrite, 13) = simData(idx).state.v;
            M(idxWrite, 14) = simData(idx).state.w;
            M(idxWrite, 15) = simData(idx).state.p;
            M(idxWrite, 16) = simData(idx).state.q;
            M(idxWrite, 17) = simData(idx).state.r;
            M(idxWrite, 18) = simData(idx).state.psi;
            M(idxWrite, 19) = simData(idx).state.theta;
            M(idxWrite, 20) = simData(idx).state.phi;
            M(idxWrite, 21) = simData(idx).state.xE;
            M(idxWrite, 22) = simData(idx).state.yE;
            M(idxWrite, 23) = simData(idx).state.zE;
            M(idxWrite, 24) = simData(idx).state.vX;
            M(idxWrite, 25) = simData(idx).state.vY;
            M(idxWrite, 26) = simData(idx).state.vZ;
            M(idxWrite, 27) = simData(idx).state.aX;
            M(idxWrite, 28) = simData(idx).state.aY;
            M(idxWrite, 29) = simData(idx).state.aZ;
            
            M(idxWrite, 30) = simData(idx).forceMoment.netForceVecInB(1);
            M(idxWrite, 31) = simData(idx).forceMoment.netForceVecInB(2);
            M(idxWrite, 32) = simData(idx).forceMoment.netForceVecInB(3);
            M(idxWrite, 33) = simData(idx).forceMoment.liftVecInB(1);
            M(idxWrite, 34) = simData(idx).forceMoment.liftVecInB(2);
            M(idxWrite, 35) = simData(idx).forceMoment.liftVecInB(3);
            M(idxWrite, 36) = simData(idx).forceMoment.dragVecInB(1);
            M(idxWrite, 37) = simData(idx).forceMoment.dragVecInB(2);
            M(idxWrite, 38) = simData(idx).forceMoment.dragVecInB(3);
            M(idxWrite, 39) = simData(idx).forceMoment.weightVecInB(1);
            M(idxWrite, 40) = simData(idx).forceMoment.weightVecInB(2);
            M(idxWrite, 41) = simData(idx).forceMoment.weightVecInB(3);
            M(idxWrite, 42) = simData(idx).forceMoment.momentVecInB(1);
            M(idxWrite, 43) = simData(idx).forceMoment.momentVecInB(2);
            M(idxWrite, 44) = simData(idx).forceMoment.momentVecInB(3);
            
            if idx > 1
                M(idxWrite, 45) = simData(idx).FC.dt;
                M(idxWrite, 46) = simData(idx).FC.phase;
                M(idxWrite, 47) = simData(idx).FC.Sensors.truthSensor.xE;
                M(idxWrite, 48) = simData(idx).FC.Sensors.truthSensor.yE;
                M(idxWrite, 49) = simData(idx).FC.Sensors.truthSensor.zE;
                M(idxWrite, 50) = simData(idx).FC.Sensors.truthSensor.xEdot;
                M(idxWrite, 51) = simData(idx).FC.Sensors.truthSensor.yEdot;
                M(idxWrite, 52) = simData(idx).FC.Sensors.truthSensor.zEdot;
                M(idxWrite, 53) = simData(idx).FC.Sensors.truthSensor.u;
                M(idxWrite, 54) = simData(idx).FC.Sensors.truthSensor.v;
                M(idxWrite, 55) = simData(idx).FC.Sensors.truthSensor.w;
                M(idxWrite, 56) = simData(idx).FC.Sensors.truthSensor.phi;
                M(idxWrite, 57) = simData(idx).FC.Sensors.truthSensor.theta;
                M(idxWrite, 58) = simData(idx).FC.Sensors.truthSensor.psi;
                M(idxWrite, 59) = simData(idx).FC.Sensors.truthSensor.p;
                M(idxWrite, 60) = simData(idx).FC.Sensors.truthSensor.q;
                M(idxWrite, 61) = simData(idx).FC.Sensors.truthSensor.r;
                M(idxWrite, 62) = simData(idx).FC.Sensors.truthSensor.aoa;
                M(idxWrite, 63) = simData(idx).FC.Sensors.truthSensor.beta;
                M(idxWrite, 64:66) = simData(idx).FC.Sensors.IMU.accel.trueBias';
                M(idxWrite, 67:69) = simData(idx).FC.Sensors.IMU.accel.specificForceMsmt';
                M(idxWrite, 70:72) = simData(idx).FC.Sensors.IMU.gyro.trueBias';
                M(idxWrite, 73:75) = simData(idx).FC.Sensors.IMU.gyro.omegaMsmt';
                M(idxWrite, 76:78) = simData(idx).FC.Sensors.GPS.posMsmt';
                M(idxWrite, 79:81) = simData(idx).FC.Sensors.GPS.velMsmt';

                M(idxWrite, 82:84) = simData(idx).FC.Navigation.Filters.InertialNavSoln.rpyhat';
                M(idxWrite, 85:87) = simData(idx).FC.Navigation.Filters.InertialNavSoln.rhat';
                M(idxWrite, 88:90) = simData(idx).FC.Navigation.Filters.InertialNavSoln.vhat';
                M(idxWrite, 91:105) = simData(idx).FC.Navigation.Filters.IntegratedNavSoln.xhat';
                M(idxWrite, 106:120) = simData(idx).FC.Navigation.Filters.IntegratedNavSoln.stdev';
                M(idxWrite, 121:123) = simData(idx).FC.Navigation.Filters.IntegratedNavSoln.biasEstimates.accel';
                M(idxWrite, 124:126) = simData(idx).FC.Navigation.Filters.IntegratedNavSoln.biasEstimates.gyro';
                M(idxWrite, 127) = simData(idx).FC.Navigation.Filters.IntegratedNavSoln.nextUpdateTime;
                M(idxWrite, 128) = simData(idx).FC.Navigation.Filters.IntegratedNavSoln.gpsMeasurement.t;
                M(idxWrite, 129:131) = simData(idx).FC.Navigation.Filters.IntegratedNavSoln.gpsMeasurement.pos';
                M(idxWrite, 132:134) = simData(idx).FC.Navigation.Filters.IntegratedNavSoln.gpsMeasurement.vel';
                
                M(idxWrite, 135) = simData(idx).FC.Navigation.ControlVariables.crossTrackError;
                M(idxWrite, 136) = simData(idx).FC.Navigation.ControlVariables.crossTrackErrorRate;
                M(idxWrite, 137) = simData(idx).FC.Navigation.ControlVariables.LonVars.alpha;
                M(idxWrite, 138) = simData(idx).FC.Navigation.ControlVariables.LonVars.q;
                M(idxWrite, 139) = simData(idx).FC.Navigation.ControlVariables.LonVars.theta;
                M(idxWrite, 140) = simData(idx).FC.Navigation.ControlVariables.LatVars.beta;
                M(idxWrite, 141) = simData(idx).FC.Navigation.ControlVariables.LatVars.p;
                M(idxWrite, 142) = simData(idx).FC.Navigation.ControlVariables.LatVars.r;
                M(idxWrite, 143) = simData(idx).FC.Navigation.ControlVariables.LatVars.phi;
                M(idxWrite, 144) = simData(idx).FC.Navigation.ControlVariables.airspeed;
                M(idxWrite, 145:147) = simData(idx).FC.Navigation.ControlVariables.posE;
                M(idxWrite, 148:150) = simData(idx).FC.Navigation.ControlVariables.velE;
                M(idxWrite, 151:153) = simData(idx).FC.Navigation.ControlVariables.posTaem;
                M(idxWrite, 154:156) = simData(idx).FC.Navigation.ControlVariables.velTaem;
                M(idxWrite, 157) = simData(idx).FC.Navigation.ControlVariables.rHac;
                M(idxWrite, 158) = simData(idx).FC.Navigation.ControlVariables.psiHac;
                M(idxWrite, 159) = simData(idx).FC.Navigation.ControlVariables.rdotHac;
                M(idxWrite, 160) = simData(idx).FC.Guidance.Gains.Vnav.KP;
                M(idxWrite, 161) = simData(idx).FC.Guidance.Gains.Vnav.KI;
                M(idxWrite, 162) = simData(idx).FC.Guidance.Gains.Vnav.KD;
                M(idxWrite, 163) = simData(idx).FC.Guidance.Gains.Lnav.KP;
                M(idxWrite, 164) = simData(idx).FC.Guidance.Gains.Lnav.KI;
                M(idxWrite, 165) = simData(idx).FC.Guidance.Gains.Lnav.KD;
                M(idxWrite, 166) = simData(idx).FC.Guidance.Commands.altitude;
                M(idxWrite, 167) = simData(idx).FC.Guidance.Commands.theta;
                M(idxWrite, 168) = simData(idx).FC.Guidance.Commands.phi;
                M(idxWrite, 169) = simData(idx).FC.Guidance.Commands.airspeed;
                M(idxWrite, 170) = simData(idx).FC.Guidance.Commands.spdbk;
                M(idxWrite, 171) = simData(idx).FC.Guidance.Commands.spdbkControllerLevel;
                M(idxWrite, 172) = simData(idx).FC.Guidance.Integrators.altError;
                M(idxWrite, 173) = simData(idx).FC.Guidance.Integrators.crossTrackError;
                M(idxWrite, 174:176) = simData(idx).FC.Autopilot.Gains.Vnav.K;
                M(idxWrite, 177) = simData(idx).FC.Autopilot.Gains.Vnav.H;
                M(idxWrite, 178:181) = simData(idx).FC.Autopilot.Gains.Lnav.K;
                M(idxWrite, 182) = simData(idx).FC.Autopilot.Gains.Lnav.H;
                M(idxWrite, 183) = simData(idx).FC.Autopilot.Integrators.thetaError;
                M(idxWrite, 184) = simData(idx).FC.Autopilot.Integrators.phiError;
                M(idxWrite, 185) = simData(idx).FC.ControlSurfCommand.gear;
                M(idxWrite, 186) = simData(idx).FC.ControlSurfCommand.trim;
                M(idxWrite, 187) = simData(idx).FC.ControlSurfCommand.elevator;
                M(idxWrite, 188) = simData(idx).FC.ControlSurfCommand.aileron;
                M(idxWrite, 189) = simData(idx).FC.ControlSurfCommand.rudder;
                M(idxWrite, 190) = simData(idx).FC.ControlSurfCommand.spdbk;
                M(idxWrite, 191:193) = simData(idx).wind.windNED;
                M(idxWrite, 194:196) = simData(idx).wind.windUVW;
                
            end
        
        idxWrite = idxWrite + 1;
            
    end
    
    runIdStr = ['000', num2str(runId)];
    runIdStr = runIdStr(end-3:end);
    
    fh = fopen([outputFolder, 'Run', runIdStr, '.hdr'], 'w');
    fprintf(fh, dataHeaders());
    fclose(fh);
    
    fh = fopen([outputFolder, 'Run', runIdStr, '.bin'], 'w');
    fwrite(fh, M, 'double', 'l');
    fclose(fh);
    
end