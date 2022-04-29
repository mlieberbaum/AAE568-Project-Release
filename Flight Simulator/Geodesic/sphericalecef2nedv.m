function [vN, vE, vD] = sphericalecef2nedv(vX, vY, vZ, lat0, lon0)

    % Function to transform a vector from ECEF to local NED coordinates
    Rned2ecef = [-sin(lat0)*cos(lon0)      -sin(lon0)         -cos(lat0)*cos(lon0)
                 -sin(lat0)*sin(lon0)       cos(lon0)         -cos(lat0)*sin(lon0)
                  cos(lat0)                 0                 -sin(lat0)];
    
    Recef2ned = transpose(Rned2ecef);
    
    rNED = Recef2ned * [vX; vY; vZ];
    
    vN = rNED(1);
    vE = rNED(2);
    vD = rNED(3);
    
end