function [netForceVecInB, liftVecMainWingInB, dragVecTotalInB, weightVecInB, weightVecInBLegacy, momentVecInB] = aerodynamicModel(...
        state, altitudeHae, spaceplane, rho, mach, ctrlSurf, windUVW)
    
    % Function to compute forces and moments in the body frame of a
    % spaceplane in atmospheric flight
    
    
    
    %% SET VARIABLES FOR READABILITY
    u = state.u + windUVW(1);
    v = state.v + windUVW(2);
    w = state.w + windUVW(3);
    rollRate = state.p;
    pitchRate = state.q;
    yawRate = state.r;
    theta = state.theta;
    phi = state.phi;
    
    gearPos = ctrlSurf.gear;
    spdbkPos = ctrlSurf.spdbk;
    elevatorPos = ctrlSurf.elevator;
    aileronPos = ctrlSurf.aileron;
    rudderPos = ctrlSurf.rudder;
    pitchTrimPos = ctrlSurf.trim;
    
    
    
    %%
    % ---------------------------------------------------------------------
    % Aerodynamic parameters
    
    % Airspeed
    aspd = sqrt(u*u + v*v + w*w);
    
    % Dynamic Pressure
    qinf = 0.5 * rho * aspd * aspd;
    
    % Angle of Attack
    aoa = atan(w/u);
    
    % Sideslip angle
    sideslip = atan(v/u);
    
    
    
    
    %%
    % ---------------------------------------------------------------------
    % Legacy Weight Vector
    
    % Old method: only spherical gravity sources
    weight = spaceplane.mass * 398600439968871 / ((altitudeHae + 6371010.0)^2);
    weightVecInBLegacy = [-weight * sin(theta)
                           weight * cos(theta) * sin(phi)
                           weight * cos(theta) * cos(phi)];
    weightVecInB = weightVecInBLegacy;
    
    
      
         
    %%
    % ---------------------------------------------------------------------            
    % Lift Vectors
    
    % Main wing lift coefficient and lift force
    cL = interp1(spaceplane.mainWing.cLvsAlpha(:,1), spaceplane.mainWing.cLvsAlpha(:,2), aoa);
    liftForceMainWing = qinf * spaceplane.mainWing.S * cL;
    
    % Total lift force and lift vector.  Lift vector
    % direction in the body frame is [sin(aoa), 0, -cos(aoa)]
    liftVecVerticalInBHat = [w; 0; -u] ./ sqrt(u^2 + w^2);
    liftVecMainWingInB = liftVecVerticalInBHat .* liftForceMainWing;
              
    % Compute the tail lift (used for moment calculations).  Tail lift
    % vector direction is  [sin(sideslip); cos(sideslip); 0]
    cLTail = interp1(spaceplane.tail.clVsBeta(:,1), spaceplane.tail.clVsBeta(:,2), sideslip);
    liftForceTail = -qinf * spaceplane.tail.S * cLTail;          % Negative sign because a positive sideslip angle produces a negative force in xBody
    liftVecHorizontalInBHat = [-v; u; 0] ./ sqrt(u^2+v^2);
    liftVecTailInB = liftForceTail * liftVecHorizontalInBHat;
    
    
    
    %%
    % ---------------------------------------------------------------------
    % Drag Vectors
    
    % Global wing/tail drag parameters
    if mach <= spaceplane.mainWing.M1
        cd_wave = 0;
    elseif mach >= spaceplane.mainWing.M1 && mach < spaceplane.mainWing.M2
        cd_wave = spaceplane.mainWing.cdWave .* ((mach - spaceplane.mainWing.M1)/(spaceplane.mainWing.M2 - spaceplane.mainWing.M1));
    elseif mach >= spaceplane.mainWing.M2 && mach < spaceplane.mainWing.M3
        cd_wave = spaceplane.mainWing.cdWave;
    elseif mach >= spaceplane.mainWing.M3
        cd_wave = spaceplane.mainWing.cdWave .* ( 0.4 / sqrt(mach^2 - 1) );
    end
    
    
    % Drag acts in the direction of the freestream airflow
    dragHat = [u/aspd; v/aspd; w/aspd];
    
    % Main wing drag coefficient, drag, and drag vector
    cdMainWing = spaceplane.mainWing.cD0 + ((cL*cL)/(pi * spaceplane.mainWing.e * spaceplane.mainWing.AR)) + cd_wave;
    dragForceMainWing = qinf * spaceplane.mainWing.S * cdMainWing;
    dragVecMainWingInB = -dragForceMainWing * dragHat;
    
    % Tail drag coefficient, drag, and drag vector
    cdTail = spaceplane.tail.cd0 + ((cLTail * cLTail) / (pi * spaceplane.tail.e * spaceplane.tail.AR)) + cd_wave;
    dragForceTail = qinf * spaceplane.tail.S * cdTail;
    dragVecTailInB = -dragForceTail .* dragHat;
    
    % Gear drag force and drag vector
    gearDrag = qinf * gearPos * spaceplane.gear.F;
    gearDragVecInB = -gearDrag .* dragHat;
    gearMoment = cross(spaceplane.gear.momentArm, gearDragVecInB);
    
    % Speedbrake drag force and drag vector
    spdbkDrag = qinf * spdbkPos * spaceplane.speedbrake.F;
    spdbkDragVecInB = -spdbkDrag .* dragHat;
    spdbkMoment = cross(spaceplane.speedbrake.momentArm, spdbkDragVecInB);
    
    % Total Drag Vector in the Body Frame
    dragVecTotalInB = dragVecMainWingInB + dragVecTailInB + gearDragVecInB + spdbkDragVecInB;
    
    
    
    
    
    
    
    
    
    % ---------------------------------------------------------------------
    % Moment Vector
    
                    
    % Begin with the rotational drag parameters
    pitchDampingMoment = -qinf * spaceplane.dampingMoment.Sref * spaceplane.dampingMoment.pitch * pitchRate;     % A positive pitch rate produces a negative pitch damping moment
    rollDampingMoment  = -qinf * spaceplane.dampingMoment.Sref * spaceplane.dampingMoment.roll * rollRate;       % A positive roll rate produces a negative roll damping moment
    yawDampingMoment   = -qinf * spaceplane.dampingMoment.Sref * spaceplane.dampingMoment.yaw * yawRate;         % A positive yaw rate produces a negative yaw damping moment
    momentVecRotDrag = [rollDampingMoment
                        pitchDampingMoment
                        yawDampingMoment];
                
                
                
    % Add the main wing pitch moment.  This is entirely a pitching moment
    % in the y-body direction, from the main wing airfoil.  This is from
    % the equation M = qinf * S * c * cM
    cMa = interp1(spaceplane.mainWing.cMvsAlpha(:,1), spaceplane.mainWing.cMvsAlpha(:,2), aoa);
    mainWingPitchingMoment = qinf * spaceplane.mainWing.S * spaceplane.mainWing.c * cMa;
    momentVecMainWingPitch = [0
                              mainWingPitchingMoment
                              0];
    
    
    % Add the moments from the tails
    cMt = interp1(spaceplane.tail.cmVsBeta(:,1), spaceplane.tail.cmVsBeta(:,2), sideslip);
    tailYawMoment = qinf * spaceplane.tail.S * spaceplane.tail.c * cMt;
    momentVecTailYaw = [0
                        0
                        tailYawMoment];
                               
                               
                               
    % Now do the moments from the lift and drag vectors from the tail.
    % These follow the equation M = r X F
    
    % Tail lift force pitching moment
    momentVecTailLift = cross(spaceplane.tail.momentArm, liftVecTailInB);
    
    % Tail drag force pitching moment
    momentVecTailDrag = cross(spaceplane.tail.momentArm, dragVecTailInB);                           
    
                
    
    
    % Finally, do the control surfaces.
    
    % Elevators
    forceElevators = qinf * spaceplane.elevator.S * elevatorPos * spaceplane.elevator.clFull;                % elevatorPos is positive when pulling back on the stick, creating a positive z force
    forceVecElevators = forceElevators .* liftVecVerticalInBHat;
    elevatorPitchingMoment = -cross(spaceplane.elevator.momentArm, forceVecElevators);
    
    dragForceElevators = qinf * spaceplane.elevator.S * abs(elevatorPos);
    dragForceVecElevators = -dragForceElevators .* dragHat;
    elevatorDragPitchingMoment = cross(spaceplane.elevator.momentArm, dragForceVecElevators);
    
    % Left Aileron
    forceLeftAileron = qinf * spaceplane.leftAileron.S * spaceplane.leftAileron.clFull * aileronPos;         % Positive because... 
    forceVecLeftAileron = forceLeftAileron .* liftVecVerticalInBHat;
    leftAileronMoment = cross(spaceplane.leftAileron.momentArm, forceVecLeftAileron);
    
    dragForceLeftAileron = qinf * spaceplane.leftAileron.S * abs(aileronPos);
    dragForceVecLeftAileron = -dragForceLeftAileron .* dragHat;
    leftAileronDragMoment = cross(spaceplane.leftAileron.momentArm, dragForceVecLeftAileron);
    
    % Right Aileron
    forceRightAileron = -qinf * spaceplane.rightAileron.S * spaceplane.rightAileron.clFull * aileronPos;     % Negative because...
    forceVecRightAileron = forceRightAileron .* liftVecVerticalInBHat;
    rightAileronMoment = cross(spaceplane.rightAileron.momentArm, forceVecRightAileron);
    
    dragForceRightAileron = qinf * spaceplane.rightAileron.S * abs(aileronPos);
    dragForceVecRightAileron = -dragForceRightAileron .* dragHat;
    rightAileronDragMoment = cross(spaceplane.rightAileron.momentArm, dragForceVecRightAileron);

    % Rudders
    forceRudder = -qinf * spaceplane.rudder.S * spaceplane.rudder.clFull * rudderPos;                         % rudderPos is positive when commanding a right yaw, producing a negative force
    forceVecRudder = forceRudder * liftVecHorizontalInBHat;
    rudderYawMoment = cross(spaceplane.rudder.momentArm, forceVecRudder);     
    
    dragForceRudders = qinf * spaceplane.rudder.S * abs(rudderPos);
    dragForceVecRudders = -dragForceRudders .* dragHat;
    rudderDragPitchingMoment = cross(spaceplane.rudder.momentArm, dragForceVecRudders);

    % Elevator Trim
    liftForceTrim = -qinf * spaceplane.pitchTrim.S * pitchTrimPos * spaceplane.pitchTrim.clFull; 
    liftForceVecTrim = liftForceTrim * liftVecVerticalInBHat;
    trimMoment = cross(spaceplane.pitchTrim.momentArm, liftForceVecTrim);
    
    dragForceTrim = qinf * spaceplane.pitchTrim.S * abs(pitchTrimPos);
    dragForceVecTrim = -dragForceTrim .* dragHat;
    trimDragMoment = cross(spaceplane.pitchTrim.momentArm, dragForceVecTrim);
    
    
    
    
                               
    
         
    %%
    % ---------------------------------------------------------------------
    % Net Force Vector
    netForceVecInB = weightVecInB + liftVecMainWingInB + liftVecTailInB + dragVecMainWingInB + dragVecTailInB + ...
        forceVecElevators + forceVecLeftAileron + forceVecRightAileron + forceVecRudder + dragForceVecElevators + ...
        dragForceVecLeftAileron + dragForceVecRightAileron + dragForceVecRudders + liftForceVecTrim + dragForceVecTrim + ...
        spdbkDragVecInB + gearDragVecInB;
    
    
    
    %% 
    % ---------------------------------------------------------------------
    % Net Moment Vector
    momentVecInB = momentVecTailLift + momentVecTailDrag + momentVecTailYaw + ...
                   leftAileronMoment + rightAileronMoment + elevatorPitchingMoment + rudderYawMoment + trimMoment + ...
                   momentVecRotDrag + momentVecMainWingPitch + elevatorDragPitchingMoment + leftAileronDragMoment + ...
                   rightAileronDragMoment + rudderDragPitchingMoment + trimDragMoment + spdbkMoment + gearMoment;
    
    
    
end