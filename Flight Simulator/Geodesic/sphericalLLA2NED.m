function [N, E, D] = sphericalLLA2NED(lat, lon, alt, lat0, lon0, alt0)

    
    [X, Y, Z] = sphericalLLA2ECEF(lat, lon, alt);
    [N, E, D] = sphericalECEF2NED(X, Y, Z, lat0, lon0, alt0);
    
    
end
    
    