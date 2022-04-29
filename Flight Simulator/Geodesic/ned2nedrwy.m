function [N, E, D] = ned2nedrwy(Ntrue, Etrue, Dtrue, rwyHeadingDegrees)

    % Function to convert NED to runway NED, simply by rotating about the D
    % axis to the correct runway heading
    th = rwyHeadingDegrees * pi / 180;
    
    M = [ cos(th)   sin(th)   0
         -sin(th)   cos(th)   0
          0         0         1 ];
      
    rwyNED = M * [Ntrue; Etrue; Dtrue];
    
    N = rwyNED(1);
    E = rwyNED(2);
    D = rwyNED(3);
    
end