function FC = flightComputerStepUpdate(FC, curState, randomNoise, curTime)

    % Step update function for the flight computer
    
    %% SENSORS STEP UPDATE
    FC = sensorsStepUpdate(FC, curState, randomNoise);
    
    
    
    %% NAV STEP UPDATE
    FC = navStepUpdate(FC, curTime);
    
    
    
    %% GUIDANCE STEP UPDATE
    if curTime >= 0.03
        FC = guidanceStepUpdate(FC);
    end
    
    
    
    %% AUTOPILOT STEP UPDATE
    if curTime >= 0.05 && (curTime - FC.Autopilot.nextUpdateTime) > -1e-8
        FC = autopilotStepUpdate(FC);
        FC.Autopilot.nextUpdateTime = curTime + FC.Autopilot.updatePeriod;
    end
    
    
end