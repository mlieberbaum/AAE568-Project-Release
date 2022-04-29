function T = Tearth2body(roll, pitch, yaw)

    cphi = cos(roll);
    sphi = sin(roll);
    cth = cos(pitch);
    sth = sin(pitch);
    cpsi = cos(yaw);
    spsi = sin(yaw);
    
    T = [ cth*cpsi                     cth*spsi                     -sth
          sphi*sth*cpsi - cphi*spsi    sphi*sth*spsi + cphi*cpsi    sphi*cth 
          cpsi*sth*cphi + sphi*spsi    cphi*sth*spsi - sphi*cpsi    cphi*cth];
    
end
    
    