function outputData = findNextState(dt, state, ctrlSurf, alt, spaceplane, rhoInf, mach, windUVW, aerodynamicModelInput, sixDofModelInput)

    
    % Compute aerodynamic forces and moments
    [netForceVecInB, liftVecInB, dragVecInB, weightVecInBNonspherical, ~, netMomentVecInB] = ...
        aerodynamicModel(state, alt, spaceplane, rhoInf, mach, ctrlSurf, windUVW);
    
    
    % Run the 6-dof state propagator
    outputData.state = sixDofRigidAircraftSolverRK45(dt, state, netForceVecInB, netMomentVecInB, spaceplane);
    
    
    % Outputs
    outputData.forceMoment.netForceVecInB = netForceVecInB;
    outputData.forceMoment.liftVecInB = liftVecInB;
    outputData.forceMoment.dragVecInB = dragVecInB;
    outputData.forceMoment.weightVecInB = weightVecInBNonspherical;
    outputData.forceMoment.momentVecInB = netMomentVecInB;
    
    
end