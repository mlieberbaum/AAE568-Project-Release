function spaceplane = loadSpaceplane(inputPath, inputFile)

    params = readInputFile([inputPath, inputFile]);
    
    
    % Spaceplane physical properties
    spaceplane.mass = params.mass;
    spaceplane.Ixx = params.IxxMultiplier * spaceplane.mass;
    spaceplane.Iyy = params.IyyMultiplier * spaceplane.mass;
    spaceplane.Izz = params.IzzMultiplier * spaceplane.mass;
    
    spaceplane.Ixy = params.IxyMultiplier * spaceplane.mass;
    spaceplane.Ixz = params.IxzMultiplier * spaceplane.mass;
    spaceplane.Iyz = params.IyzMultiplier * spaceplane.mass;
	
	spaceplane.IxxMultiplier = params.IxxMultiplier;
	spaceplane.IyyMultiplier = params.IyyMultiplier;
	spaceplane.IzzMultiplier = params.IzzMultiplier;
    
    spaceplane.IxyMultiplier = params.IxyMultiplier;
	spaceplane.IxzMultiplier = params.IxzMultiplier;
    spaceplane.IyzMultiplier = params.IyzMultiplier;
    
    spaceplane.Ib = [spaceplane.Ixx   spaceplane.Ixy   spaceplane.Ixz
                     spaceplane.Ixy   spaceplane.Iyy   spaceplane.Iyz
                     spaceplane.Ixz   spaceplane.Iyz   spaceplane.Izz];
    
    spaceplane.mainWing = params.mainWing;
    spaceplane.mainWing.cLvsAlpha = dlmread([inputPath, 'MainWingLiftVsAlpha.csv']);
    spaceplane.mainWing.cMvsAlpha = dlmread([inputPath, 'MainWingMomentVsAlpha.csv']);
    spaceplane.mainWing.cLvsAlpha(:,1) = spaceplane.mainWing.cLvsAlpha(:,1) .* (pi/180);
    spaceplane.mainWing.cMvsAlpha(:,1) = spaceplane.mainWing.cMvsAlpha(:,1) .* (pi/180);
    
    spaceplane.tail = params.tail;
    spaceplane.tail.clVsBeta = dlmread([inputPath, 'TailLiftVsBeta.csv']);
    spaceplane.tail.cmVsBeta = dlmread([inputPath, 'TailMomentVsBeta.csv']);
    spaceplane.tail.clVsBeta(:,1) = spaceplane.tail.clVsBeta(:,1) .* (pi/180);
    spaceplane.tail.cmVsBeta(:,1) = spaceplane.tail.cmVsBeta(:,1) .* (pi/180);
    
    spaceplane.elevator = params.elevator;
    spaceplane.leftAileron = params.leftAileron;
    spaceplane.rightAileron = params.rightAileron;
    spaceplane.rudder = params.rudder;
    spaceplane.pitchTrim = params.pitchTrim;
    spaceplane.speedbrake = params.speedbrake;
    spaceplane.gear = params.gear;
    
    spaceplane.dampingMoment.pitch = params.pitchDampingMomentFactor;
    spaceplane.dampingMoment.roll = params.rollDampingMomentFactor;
    spaceplane.dampingMoment.yaw = params.yawDampingMomentFactor;
    spaceplane.dampingMoment.Sref = params.yCrossSection;
    
    spaceplane.longitudinalGains.K = params.longitudinalGains.K;
    spaceplane.longitudinalGains.H = params.longitudinalGains.H;
    spaceplane.lateralGains.K = params.lateralGains.K;
    spaceplane.lateralGains.H = params.lateralGains.H;
    
end