function [X, Y, Z] = sphericalLLA2ECEF(latRad, lonRad, alt)

    % Radial distance
    r = 6371010.0 + alt;
    
    X = r * cos(latRad) * cos(lonRad);
    Y = r * cos(latRad) * sin(lonRad);
    Z = r * sin(latRad);
    
end