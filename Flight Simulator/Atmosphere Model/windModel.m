function [windNED, windUVW] = windModel(curState, windDirection, windMultiplier, gustData)

    % Function to return disturbances ug, vg, and wg 
    
    % From HWM 1993, 30-60 N Latitude
    averageWind = [ 0     2
                    2     6
                    4    10
                    6    14
                    8    16
                   10  16.5
                   12    16
                   14  14.5
                   16    12
                   18     9
                   20   6.5
                   22     4
                   24     3
                   26     3
                   28   3.5
                   30     5];
    
    
    % Wind Components, Earth Frame
    altitude = -curState.zE;
    windSpeed = windMultiplier * interp1(averageWind(:,1), averageWind(:,2), altitude / 1000);
    
    xWind = -windSpeed * cos(windDirection);
    yWind = -windSpeed * sin(windDirection);
    
    
    % Gust components, Earth Frame
    if gustData(2) == 0
        xGust = 0;
        yGust = 0;
    else
        gustDir = windDirection + gustData(3);
        gustMagnitude = gustData(2) * windSpeed;
        xGust = -gustMagnitude * cos(gustDir);
        yGust = -gustMagnitude * sin(gustDir);
    end
        
    xg = xWind + xGust;
    yg = yWind + yGust;
    zg = 0;
    
    windNED = [xg; yg; zg];
    
    
    % Transform to body velocity axes
    Te2b = Tearth2body(curState.phi, curState.theta, curState.psi);
    windUVW = Te2b * windNED;
    
end
    