function [inputData, spaceplane] = applyMonteCarloPerturbations(inputData, spaceplane, mcParams)


    %% STANDARD ATMOSPHERE PERTURBATIONS
    
    gamma = 1.4;
    R = 287;
    inputData.standardAtmosphere(:, 4) = inputData.standardAtmosphere(:, 4) .* (1 + mcParams.densityMultiplier);
    inputData.standardAtmosphere(:, 3) = inputData.standardAtmosphere(:, 3) + mcParams.temperatureShift;
    inputData.standardAtmosphere(:, 5) = sqrt(gamma * R .* inputData.standardAtmosphere(:, 3));
    inputData.standardAtmosphere(:, 2) = inputData.standardAtmosphere(:, 4) .* R .* inputData.standardAtmosphere(:, 3);
    
    
    
    
    
    %% SPACEPLANE PERTURBATIONS
    spaceplane.mass = spaceplane.mass * (1 + mcParams.pmi.massScaleFactor);
    spaceplane.Ixx = spaceplane.Ixx * (1 + mcParams.pmi.IxxScaleFactor);
    spaceplane.Iyy = spaceplane.Iyy * (1 + mcParams.pmi.IyyScaleFactor);
    spaceplane.Izz = spaceplane.Izz * (1 + mcParams.pmi.IzzScaleFactor);
    spaceplane.Ixy = spaceplane.Ixy + ((spaceplane.Ixx + spaceplane.Iyy)/2) * mcParams.pmi.IxyScaleFactor;
    spaceplane.Ixz = spaceplane.Ixz + ((spaceplane.Ixx + spaceplane.Izz)/2) * mcParams.pmi.IxzScaleFactor;
    spaceplane.Iyz = spaceplane.Iyz + ((spaceplane.Iyy + spaceplane.Izz)/2) * mcParams.pmi.IyzScaleFactor;
    
    spaceplane.mainWing.cLvsAlpha(:,2) = spaceplane.mainWing.cLvsAlpha(:,2) .* mcParams.aero.mainWingLiftCurveMultiplier;
    spaceplane.mainWing.cMvsAlpha(:,2) = spaceplane.mainWing.cMvsAlpha(:,2) .* mcParams.aero.mainWingMomentCurveMultiplier;
    spaceplane.tail.clVsBeta(:,2) = spaceplane.tail.clVsBeta(:,2) .* mcParams.aero.tailLiftCurveMultiplier;
    spaceplane.tail.cmVsBeta(:,2) = spaceplane.tail.cmVsBeta(:,2) .* mcParams.aero.tailMomentCurveMultiplier;
    
    
    
end