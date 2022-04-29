function [vX, vY, vZ] = sphericalned2ecefv(vN, vE, vD, lat0, lon0)

    % Function to transform a vector from local NED coordinates to ECEF
    % coordinates
    Rned2ecef = [-sin(lat0)*cos(lon0)      -sin(lon0)         -cos(lat0)*cos(lon0)
                 -sin(lat0)*sin(lon0)       cos(lon0)         -cos(lat0)*sin(lon0)
                  cos(lat0)                 0                 -sin(lat0)];
    
    rECEF = Rned2ecef * [vN; vE; vD];
    
    vX = rECEF(1);
    vY = rECEF(2);
    vZ = rECEF(3);
    
end