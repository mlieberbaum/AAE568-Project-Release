function [rN, rE, rD] = sphericalECEF2NED(X, Y, Z, lat0, lon0, alt0)

    % Function to convert ECEF coordinates to NED coordinates
    [X0, Y0, Z0] = sphericalLLA2ECEF(lat0, lon0, alt0);
    
    drECEF = [X - X0
              Y - Y0
              Z - Z0];
          
    [rN, rE, rD] = sphericalecef2nedv(drECEF(1), drECEF(2), drECEF(3), lat0, lon0);
    
    
    
end