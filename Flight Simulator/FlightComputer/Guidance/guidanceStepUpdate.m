function FC = guidanceStepUpdate(FC)


    switch FC.phase
        
        
        case 'TAEM'
            
            FC.Guidance.Gains.Lnav.KP = -1.27273E-06*FC.Navigation.ControlVariables.airspeed + 0.001318182;
            FC.Guidance.Gains.Lnav.KI = -1.27273E-07*FC.Navigation.ControlVariables.airspeed + 0.000131818;
            FC.Guidance.Gains.Lnav.KD = -2.54545E-05*FC.Navigation.ControlVariables.airspeed + 0.026363636;

            
            gammaNominal = -atan(polyval(FC.trajectoryProps.taem.slopecmd, -FC.Navigation.ControlVariables.posTaem(1)));
            hcmd = polyval(FC.trajectoryProps.taem.hcmd, -FC.Navigation.ControlVariables.posTaem(1));
            
            FC.Guidance.Commands.airspeed = polyval(FC.trajectoryProps.taem.aspdPolynomial, -FC.Navigation.ControlVariables.posTaem(1));
            
            if FC.Navigation.ControlVariables.posTaem(1) >= 0
                FC.phase = 'HAC';
            end
            
        case 'HAC'
            
            % Feed-forward gamma value
            gammaNominal = FC.trajectoryProps.longFinal.gamma * pi/180;
            hcmd = FC.trajectoryProps.waypoints.wp3(3) - tan(gammaNominal)*...
                (FC.trajectoryProps.hac.r1*FC.Navigation.ControlVariables.psiHac + (1/3)*FC.trajectoryProps.hac.r2*(FC.Navigation.ControlVariables.psiHac)^3);
            
            FC.Guidance.Commands.airspeed = polyval(FC.trajectoryProps.hac.aspdPolynomial, FC.Navigation.ControlVariables.psiHac * 180/pi);
            
            FC.Guidance.Gains.Lnav.KP = 0.001;
            FC.Guidance.Gains.Lnav.KI = 0.0001;
            FC.Guidance.Gains.Lnav.KD = 0.02;

            if FC.Navigation.ControlVariables.psiHac < 2*pi/180
                FC.phase = 'LONG_FINAL';
            end
        

        case 'LONG_FINAL'
            
            FC.Guidance.Gains.Lnav.KP = 0.0015;
            FC.Guidance.Gains.Lnav.KI = 0.0;
            FC.Guidance.Gains.Lnav.KD = 0.02;

            gammaNominal = FC.trajectoryProps.longFinal.gamma * pi/180;
            hcmd = (FC.Navigation.ControlVariables.posE(1) - FC.trajectoryProps.longFinal.aimPoint(1)) * tan(gammaNominal);
            
            if -FC.Navigation.ControlVariables.posE(3) < 700
                FC.phase = 'PRE_FLARE';
            end
            
            FC.Guidance.Commands.airspeed = polyval(FC.trajectoryProps.longFinal.aspdPolynomial, -FC.Navigation.ControlVariables.posE(3));
            

        case 'PRE_FLARE'
            
            FC.Guidance.Gains.Vnav.KP = 0.0025*3;
            FC.Guidance.Gains.Vnav.KI = 0.001;
            FC.Guidance.Gains.Vnav.KD = 0.01;
            
            gammaNominal = atan(polyval(FC.trajectoryProps.preFlare.slopecmd, FC.Navigation.ControlVariables.posE(1)));
            hcmd = polyval(FC.trajectoryProps.preFlare.hcmd, FC.Navigation.ControlVariables.posE(1));
            
            FC.Guidance.Commands.airspeed = 200;
            
            if -FC.Navigation.ControlVariables.posE(3) < 75
                FC.phase = 'SHORT_FINAL';
            end
            

        case 'SHORT_FINAL'
            
            FC.Guidance.Gains.Vnav.KP = 0.0025*3;
            FC.Guidance.Gains.Vnav.KI = 0.001;
            FC.Guidance.Gains.Vnav.KD = 0.01;
            
            gammaNominal = FC.trajectoryProps.shortFinal.gamma * pi/180;


            % Gamma command computation
            hcmd = FC.trajectoryProps.waypoints.wp6(3) + FC.Navigation.ControlVariables.posE(1) * tan(gammaNominal);
            
            FC.ControlSurfCommand.spdbk = 0;
            
            if -FC.Navigation.ControlVariables.posE(3) < 22
                FC.phase = 'FLARE';
            end
            
            
        case 'FLARE'
            
            FC.Guidance.Gains.Vnav.KP = 0.0025*3;
            FC.Guidance.Gains.Vnav.KI = 0.001;
            FC.Guidance.Gains.Vnav.KD = 0.01;
    
            gammaNominal = atan(polyval([-2.688933092169e-08  7.75517791805424e-05 -0.0524077792830412], FC.Navigation.ControlVariables.posE(1)));
            if FC.Navigation.ControlVariables.posE(1) <= 1000
                hcmd = polyval([-8.96311030723e-09 3.87758895902712e-05 -0.0524077792830412  22.595], FC.Navigation.ControlVariables.posE(1));
            else
                hcmd = 0;
            end
            FC.ControlSurfCommand.spdbk = 0;
            
    end
    
    
    %% VNAV
    
    % Current altitude error
    h = -FC.Navigation.ControlVariables.posE(3);
    herr = hcmd - h;
    FC.Guidance.Integrators.altError = FC.Guidance.Integrators.altError + herr*FC.dt;
    
    
    % Altitude Error Rate
    FC.Guidance.Commands.altitude = hcmd;
    
    vertSpeedNominal = norm(FC.Navigation.ControlVariables.velE) * sin(gammaNominal);
    gammaActual = atan(-FC.Navigation.ControlVariables.velE(3) / norm([FC.Navigation.ControlVariables.velE(1:2); 0]));
    vertSpeedActual = norm(FC.Navigation.ControlVariables.velE) * sin(gammaActual);
    verticalErrorRate = vertSpeedNominal - vertSpeedActual;
    
    
    % Pitch Command
    newPitchCommand = FC.Guidance.Gains.Vnav.KP*herr + FC.Guidance.Gains.Vnav.KI*FC.Guidance.Integrators.altError + FC.Guidance.Gains.Vnav.KD*verticalErrorRate + gammaNominal;
    minPitchCommand = FC.Guidance.Commands.theta - FC.Guidance.CommandRateLimits.theta * FC.dt;
    maxPitchCommand = FC.Guidance.Commands.theta + FC.Guidance.CommandRateLimits.theta * FC.dt;
    FC.Guidance.Commands.theta = bound(newPitchCommand, minPitchCommand, maxPitchCommand);
    
    
    % For nav system vertical error rate calculation
    FC.Guidance.Commands.gammaNominal = gammaNominal;

    
    
    %% LNAV
    
    
    if ~strcmp(FC.phase, 'HAC')
        newBankCommand = FC.Guidance.Gains.Lnav.KP*FC.Navigation.ControlVariables.crossTrackError + FC.Guidance.Gains.Lnav.KD*FC.Navigation.ControlVariables.crossTrackErrorRate;
    else
        FC.Guidance.Integrators.crossTrackError = FC.Guidance.Integrators.crossTrackError + (FC.Navigation.ControlVariables.crossTrackError * FC.dt);
        newBankCommand = -FC.Guidance.Gains.Lnav.KP*FC.Navigation.ControlVariables.crossTrackError + ...
            FC.Guidance.Gains.Lnav.KD*FC.Navigation.ControlVariables.crossTrackErrorRate + ...
            -FC.Guidance.Gains.Lnav.KI*FC.Guidance.Integrators.crossTrackError;
    end
    
    minBankCommand = FC.Guidance.Commands.phi - FC.Guidance.CommandRateLimits.phi * FC.dt;
    maxBankCommand = FC.Guidance.Commands.phi + FC.Guidance.CommandRateLimits.phi * FC.dt;
    FC.Guidance.Commands.phi = bound(newBankCommand, minBankCommand, maxBankCommand);
    
end