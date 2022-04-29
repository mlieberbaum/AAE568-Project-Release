function monteCarloPlots(DATA, colors, plotFolder, saveBool)    

    r2d = 180.0 / pi;
    nRuns = numel(DATA);


    %% PLOTS

    % Plot size/position
    figurePos = [670 430 1090 690];

    % Control Surfaces
    makeNewPlot(figurePos, 'Time (s)', '', 'Elevators');
    for idx = 1:nRuns
        patchline(DATA(idx).t, DATA(idx).ctrlSurf.elevator, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    plot([0 400], [1 1], 'r');
    plot([0 400], [-1 -1], 'r');
    ylim([-1.2 1.2])
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'Elevators.png'], 'Resolution', 400);
        close(gcf);
    end
    
    makeNewPlot(figurePos, 'Time (s)', '', 'Ailerons');
    for idx = 1:nRuns
        patchline(DATA(idx).t, DATA(idx).ctrlSurf.aileron, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    plot([0 400], [1 1], 'r');
    plot([0 400], [-1 -1], 'r');
    ylim([-1.2 1.2])
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'Ailerons.png'], 'Resolution', 400);
        close(gcf);
    end
    
    makeNewPlot(figurePos, 'Time (s)', '', 'Rudder');
    for idx = 1:nRuns
        patchline(DATA(idx).t, DATA(idx).ctrlSurf.rudder, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    plot([0 400], [1 1], 'r');
    plot([0 400], [-1 -1], 'r');
    ylim([-1.2 1.2])
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'Rudders.png'], 'Resolution', 400);
        close(gcf);
    end
    
    makeNewPlot(figurePos, 'Time (s)', '', 'Speed Brake');
    for idx = 1:nRuns
        patchline(DATA(idx).t, DATA(idx).ctrlSurf.spdbk, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    plot([0 400], [1 1], 'r');
    plot([0 400], [0 0], 'r');
    ylim([-0.2 1.2])
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'SpeedBrake.png'], 'Resolution', 400);
        close(gcf);
    end
    
    
    % Aerodynamics
    
    
    


    % Forces
    makeNewPlot(figurePos, 'Time (s)', 'Newtons', 'Net Force, X Body');
    for idx = 1:nRuns
        patchline(DATA(idx).t, DATA(idx).forceMoment.netForceVecInB.X, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'NetForceX.png'], 'Resolution', 400);
        close(gcf);
    end
    
    makeNewPlot(figurePos, 'Time (s)', 'Newtons', 'Net Force, Y Body');
    for idx = 1:nRuns
        patchline(DATA(idx).t, DATA(idx).forceMoment.netForceVecInB.Y, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'NetForceY.png'], 'Resolution', 400);
        close(gcf);
    end
    
    makeNewPlot(figurePos, 'Time (s)', 'Newtons', 'Net Force, Z Body');
    for idx = 1:nRuns
        patchline(DATA(idx).t, DATA(idx).forceMoment.netForceVecInB.Z, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'NetForceZ.png'], 'Resolution', 400);
        close(gcf);
    end

    
    
    % Moments
    makeNewPlot(figurePos, 'Time (s)', 'N m', 'Net Moment, X Body');
    for idx = 1:nRuns
        patchline(DATA(idx).t, DATA(idx).forceMoment.momentVecInB.X, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'NetMomentX.png'], 'Resolution', 400);
        close(gcf);
    end

    makeNewPlot(figurePos, 'Time (s)', 'N m', 'Net Moment, Y Body');
    for idx = 1:nRuns
        patchline(DATA(idx).t, DATA(idx).forceMoment.momentVecInB.Y, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'NetMomentY.png'], 'Resolution', 400);
        close(gcf);
    end

    makeNewPlot(figurePos, 'Time (s)', 'N m', 'Net Moment, Z Body');
    for idx = 1:nRuns
        patchline(DATA(idx).t, DATA(idx).forceMoment.momentVecInB.Z, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'NetMomentZ.png'], 'Resolution', 400);
        close(gcf);
    end



    % Body Rates
    makeNewPlot(figurePos, 'Time (s)', 'deg/s', 'p (Roll Rate');
    for idx = 1:nRuns
        patchline(DATA(idx).t, DATA(idx).state.p, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'p.png'], 'Resolution', 400);
        close(gcf);
    end

    makeNewPlot(figurePos, 'Time (s)', 'deg/s', 'q (Pitch Rate');
    for idx = 1:nRuns
        patchline(DATA(idx).t, DATA(idx).state.q, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'q.png'], 'Resolution', 400);
        close(gcf);
    end

    makeNewPlot(figurePos, 'Time (s)', 'deg/s', 'r (Yaw Rate');
    for idx = 1:nRuns
        patchline(DATA(idx).t, DATA(idx).state.r, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'r.png'], 'Resolution', 400);
        close(gcf);
    end



    % Euler Angles
    makeNewPlot(figurePos, 'Time (s)', 'deg', 'Bank Angle \phi');
    for idx = 1:nRuns
        patchline(DATA(idx).t, DATA(idx).state.phi * r2d, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'phi.png'], 'Resolution', 400);
        close(gcf);
    end

    makeNewPlot(figurePos, 'Time (s)', 'deg', 'Pitch Angle \theta');
    for idx = 1:nRuns
        patchline(DATA(idx).t, DATA(idx).state.theta * r2d, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'theta.png'], 'Resolution', 400);
        close(gcf);
    end

    makeNewPlot(figurePos, 'Time (s)', 'deg', 'Yaw Angle \psi');
    for idx = 1:nRuns
        patchline(DATA(idx).t, DATA(idx).state.psi * r2d, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'psi.png'], 'Resolution', 400);
        close(gcf);
    end



    % Body Velocities
    makeNewPlot(figurePos, 'Time (s)', 'm/s', 'u Velocity')
    for idx = 1:nRuns
        patchline(DATA(idx).t, DATA(idx).state.u, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'u.png'], 'Resolution', 400);
        close(gcf);
    end

    makeNewPlot(figurePos, 'Time (s)', 'm/s', 'v Velocity')
    for idx = 1:nRuns
        patchline(DATA(idx).t, DATA(idx).state.v, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'v.png'], 'Resolution', 400);
        close(gcf);
    end

    makeNewPlot(figurePos, 'Time (s)', 'm/s', 'w Velocity')
    for idx = 1:nRuns
        patchline(DATA(idx).t, DATA(idx).state.w, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'w.png'], 'Resolution', 400);
        close(gcf);
    end
    
    
    
    % Flight Computer Plots
    makeNewPlot(figurePos, 'Time (s)', 'm', 'Altitude Error (h_c_m_d - h)');
    for idx = 1:nRuns
        patchline(DATA(idx).t(10:end), DATA(idx).FC.Guidance.Commands.altitude(10:end) + DATA(idx).state.zE(10:end), 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'AltitudeError.png'], 'Resolution', 400);
        close(gcf);
    end
    
    makeNewPlot(figurePos, 'Time (s)', 'Degrees', 'Pitch Angle Error (\theta_c_m_d - \theta)');
    for idx = 1:nRuns
        patchline(DATA(idx).t(2:end), (DATA(idx).FC.Guidance.Commands.theta(2:end) - DATA(idx).FC.Navigation.ControlVariables.LonVars.theta(2:end)) .* r2d, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'PitchError.png'], 'Resolution', 400);
        close(gcf);
    end
    
    makeNewPlot(figurePos, 'Time (s)', 'Degrees', 'Bank Angle Error (\phi_c_m_d - \phi)');
    for idx = 1:nRuns
        patchline(DATA(idx).t(2:end), (DATA(idx).FC.Guidance.Commands.phi(2:end) - DATA(idx).FC.Navigation.ControlVariables.LatVars.phi(2:end)) .* r2d, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'RollError.png'], 'Resolution', 400);
        close(gcf);
    end
    
    makeNewPlot(figurePos, 'Time (s)', 'm', 'Nav x_E Position Error');
    for idx = 1:nRuns
        patchline(DATA(idx).t, DATA(idx).state.xE - DATA(idx).FC.Navigation.Filters.InertialNavSoln.rhat.X, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'NavErrorPosX.png'], 'Resolution', 400);
        close(gcf);
    end
    
    makeNewPlot(figurePos, 'Time (s)', 'm', 'Nav y_E Position Error');
    for idx = 1:nRuns
        patchline(DATA(idx).t, DATA(idx).state.yE - DATA(idx).FC.Navigation.Filters.InertialNavSoln.rhat.Y, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'NavErrorPosY.png'], 'Resolution', 400);
        close(gcf);
    end
    
    makeNewPlot(figurePos, 'Time (s)', 'm', 'Nav z_E Position Error');
    for idx = 1:nRuns
        patchline(DATA(idx).t, DATA(idx).state.zE - DATA(idx).FC.Navigation.Filters.InertialNavSoln.rhat.Z, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'NavErrorPosZ.png'], 'Resolution', 400);
        close(gcf);
    end
    
    makeNewPlot(figurePos, 'Time (s)', 'm/s', 'Nav x_E Velocity Error');
    for idx = 1:nRuns
        patchline(DATA(idx).t, DATA(idx).state.vX - DATA(idx).FC.Navigation.Filters.InertialNavSoln.vhat.X, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'NavErrorVelX.png'], 'Resolution', 400);
        close(gcf);
    end
    
    makeNewPlot(figurePos, 'Time (s)', 'm/s', 'Nav y_E Velocity Error');
    for idx = 1:nRuns
        patchline(DATA(idx).t, DATA(idx).state.vY - DATA(idx).FC.Navigation.Filters.InertialNavSoln.vhat.Y, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'NavErrorVelY.png'], 'Resolution', 400);
        close(gcf);
    end
    
    makeNewPlot(figurePos, 'Time (s)', 'm/s', 'Nav z_E Velocity Error');
    for idx = 1:nRuns
        patchline(DATA(idx).t, DATA(idx).state.vZ - DATA(idx).FC.Navigation.Filters.InertialNavSoln.vhat.Z, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'NavErrorVelZ.png'], 'Resolution', 400);
        close(gcf);
    end
    
    makeNewPlot(figurePos, 'Time (s)', 'degrees', 'Nav Roll Error');
    for idx = 1:nRuns
        patchline(DATA(idx).t, (DATA(idx).state.phi - DATA(idx).FC.Navigation.Filters.InertialNavSoln.rpyhat.X) .* 180/pi, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'NavErrorPhi.png'], 'Resolution', 400);
        close(gcf);
    end
    
    makeNewPlot(figurePos, 'Time (s)', 'degrees', 'Nav Pitch Error');
    for idx = 1:nRuns
        patchline(DATA(idx).t, (DATA(idx).state.theta - DATA(idx).FC.Navigation.Filters.InertialNavSoln.rpyhat.Y) .* 180/pi, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'NavErrorTheta.png'], 'Resolution', 400);
        close(gcf);
    end
    
    makeNewPlot(figurePos, 'Time (s)', 'degrees', 'Nav Yaw Error');
    for idx = 1:nRuns
        patchline(DATA(idx).t, unwrap(DATA(idx).state.psi - DATA(idx).FC.Navigation.Filters.InertialNavSoln.rpyhat.Z) .* 180/pi, 'FaceColor', colors(idx,:), 'EdgeColor', colors(idx,:), 'FaceAlpha', .3, 'EdgeAlpha', .3);
    end
    if saveBool == true
        exportgraphics(gcf, [plotFolder, 'NavErrorPsi.png'], 'Resolution', 400);
        close(gcf);
    end
    
    makeNewPlot(figurePos, 'm/s', '', 'Vertical Speed at Touchdown');
    touchdownVerticalSpeed = nan(nRuns,1);
    for idx = 1:nRuns
        touchdownVerticalSpeed(idx) = DATA(idx).state.vZ(end);
    end
    
    histogram(gca, touchdownVerticalSpeed, 25);
    
    makeNewPlot(figurePos, 'm', '', 'Horizontal Distance from Runway Centerline at Touchdown');
    touchdownHorizontalDistance = nan(nRuns,1);
    for idx = 1:nRuns
        touchdownHorizontalDistance(idx) = DATA(idx).state.yE(end);
    end
    
    histogram(gca, touchdownHorizontalDistance, 25);

end