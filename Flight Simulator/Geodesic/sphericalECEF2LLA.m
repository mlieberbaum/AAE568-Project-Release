function [lat, lon, alt] = sphericalECEF2LLA(xECEF, yECEF, zECEF)

    % Function to convert ECEF to lat/lon/alt, assuming a spherical datum
    rVec = [xECEF; yECEF; zECEF];
    alt = norm(rVec) - 6371010.0;
    
    rGnd = rVec(1:2);
    
    lon = atan2(rGnd(2), rGnd(1));
    lat = atan(rVec(3) ./ norm(rGnd));
    
end