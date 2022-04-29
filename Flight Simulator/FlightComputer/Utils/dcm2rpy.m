function rpy = dcm2rpy(DCM)
    
    % DCM is the transformation matrix from Earth to Body
    
    
    % Pitch
    pitch = -asin(DCM(1,3));
    
    
    % Roll
    roll1 = abs(acos(DCM(3,3) / cos(pitch)));
    roll2 = -roll1;
    
    % See which roll guess is closer
    R23calc1 = sin(roll1)*cos(pitch);
    R23calc2 = sin(roll2)*cos(pitch);
    
    deltaRoll = abs([DCM(2,3)-R23calc1, DCM(2,3)-R23calc2]);
    
    if deltaRoll(1) < deltaRoll(2)
        roll = roll1;
    else
        roll = roll2;
    end
    
    
    
    % Yaw
    yaw1 = abs(acos(DCM(1,1) / cos(pitch)));
    yaw2 = -yaw1;
    
    % See which yaw guess is closer
    R12calc1 = sin(yaw1)*cos(pitch);
    R12calc2 = sin(yaw2)*cos(pitch);
    
    deltaYaw = abs([DCM(1,2)-R12calc1, DCM(1,2)-R12calc2]);
    
    if deltaYaw(1) < deltaYaw(2)
        yaw = yaw1;
    else
        yaw = yaw2;
    end
    
    rpy = [roll pitch yaw];
    
    
end