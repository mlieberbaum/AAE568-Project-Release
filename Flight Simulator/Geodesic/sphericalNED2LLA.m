function [lat, lon, alt] = sphericalNED2LLA(N, E, D, lat0, lon0, alt0)

    % NED to LLA
    [xT, yT, zT] = sphericalned2ecefv(N, E, D, lat0, lon0);
    [x0, y0, z0] = sphericalLLA2ECEF(lat0, lon0, alt0);
    
    rECEF = [xT + x0, yT + y0, zT + z0];
    
    [lat, lon, alt] = sphericalECEF2LLA(rECEF(1), rECEF(2), rECEF(3));
    
    
end