function makePlots(inputData, DATA, trajectoryParameters)    

    r2d = 180.0 / pi;


    %% PLOTS

    % Plot size/position
    plotPos = [670 430 1090 690];

    % Control Surfaces
    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)
    plot([DATA.t], [DATA.ctrlSurf.spdbk]);
    plot([DATA.t], [DATA.ctrlSurf.elevator]);
    plot([DATA.t], [DATA.ctrlSurf.aileron]);
    plot([DATA.t], [DATA.ctrlSurf.rudder]);
    legend('Speedbrake', 'Elevator', 'Aileron', 'Rudder', 'Location', 'Best');
    
    
    % Control Surface Rates
    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)
    plot(DATA.t(2:end), diff(DATA.ctrlSurf.spdbk));
    plot(DATA.t(2:end), diff(DATA.ctrlSurf.elevator));
    plot(DATA.t(2:end), diff(DATA.ctrlSurf.aileron));
    plot(DATA.t(2:end), diff(DATA.ctrlSurf.rudder));
    legend('Speedbrake', 'Elevator', 'Aileron', 'Rudder', 'Location', 'Best');


    % Forces
    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, DATA.forceMoment.netForceVecInB(:, 1));
    xlabel('Time (s)');
    ylabel('N');
    title('Net Force, X Body');


    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, DATA.forceMoment.netForceVecInB(:, 2));
    xlabel('Time (s)');
    ylabel('N');
    title('Net Force, Y Body');


    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, DATA.forceMoment.netForceVecInB(:, 3));
    xlabel('Time (s)');
    ylabel('N');
    title('Net Force, Z Body');




    % Moments
    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, DATA.forceMoment.momentVecInB(:, 1));
    xlabel('Time (s)');
    ylabel('N m');
    title('Net Moment, X Body');


    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, DATA.forceMoment.momentVecInB(:, 2));
    xlabel('Time (s)');
    ylabel('N m');
    title('Net Moment, Y Body');


    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, DATA.forceMoment.momentVecInB(:, 3));
    xlabel('Time (s)');
    ylabel('N m');
    title('Net Moment, Z Body');




    % Body Rates
    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, DATA.state.p);
    xlabel('Time (s)');
    ylabel('deg/s');
    title('p (Roll Rate)');


    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, DATA.state.q);
    xlabel('Time (s)');
    ylabel('deg/s');
    title('q (Pitch Rate)');


    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, DATA.state.r);
    xlabel('Time (s)');
    ylabel('deg/s');
    title('r (Yaw Rate)');




    % Euler Angles
    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, DATA.state.phi * r2d);
    xlabel('Time (s)');
    ylabel('deg');
    title('Bank Angle \phi');


    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, DATA.state.theta * r2d);
    xlabel('Time (s)');
    ylabel('deg');
    title('Pitch Angle \theta');


    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, DATA.state.psi * r2d);
    xlabel('Time (s)');
    ylabel('deg');
    title('Heading Angle \psi');




    % Body Velocities
    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, DATA.state.u);
    xlabel('Time (s)');
    ylabel('m/s');
    title('u Velocity');


    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, DATA.state.v);
    xlabel('Time (s)');
    ylabel('m/s');
    title('v Velocity');


    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, DATA.state.w);
    xlabel('Time (s)');
    ylabel('m/s');
    title('w Velocity');
    
    
    
    % Overhead View
    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)
    
    plot(DATA.state.yE, DATA.state.xE)
    
    xlabel('East')
    ylabel('North')
    title('Overhead View')
    axis equal
    
    
    % Cross track error
    figure
    hold on
    grid on
    set(gcf, 'Color', 'w')
    set(gcf, 'Position', plotPos)
    
    plot(DATA.t, DATA.FC.Navigation.ControlVariables.crossTrackError);
    xlabel('Time (s)')
    ylabel('m')
    title('Cross Track Error')
    
    
    % Flight Computer Plots
    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, DATA.FC.Guidance.Commands.altitude);
    plot(DATA.t, -DATA.state.zE)

    xlabel('Time (s)')
    ylabel('meters')
    title('Altitude Command vs. Altitude')
    legend('h_c_m_d', 'h', 'Location', 'Best');


    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t(10:end), DATA.FC.Guidance.Commands.altitude(10:end) + DATA.state.zE(10:end));

    xlabel('Time (s)')
    ylabel('meters')
    title('Altitude Error (Cmd - Pos)')


    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, DATA.FC.Guidance.Commands.theta .* r2d)
    plot(DATA.t, DATA.FC.Navigation.ControlVariables.LonVars.theta .* r2d)

    xlabel('Time (s)')
    ylabel('Degrees')
    title('Pitch Angle Command')
    legend('\theta_c_m_d', '\theta', 'Location', 'Best')


    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, (DATA.FC.Guidance.Commands.theta - DATA.FC.Navigation.ControlVariables.LonVars.theta) .* r2d)

    xlabel('Time (s)')
    ylabel('Degrees')
    title('Pitch Path Angle Error')


    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, DATA.FC.Guidance.Commands.phi .* r2d)
    plot(DATA.t, DATA.FC.Navigation.ControlVariables.LatVars.phi .* r2d);
    legend('\phi_c_m_d', 'phi', 'Location', 'Best');



    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, DATA.state.vE)
    plot(DATA.t, DATA.FC.Guidance.Commands.airspeed)
    yyaxis right
    plot(DATA.t, DATA.FC.ControlSurfCommand.spdbk)
    legend('Airspeed', 'Airspeed Command', 'Speedbrake')



    % Nav Solution Position Errors
    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, DATA.state.xE - DATA.FC.Navigation.Filters.InertialNavSoln.rhat(:,1))
    plot(DATA.t, 3*DATA.FC.Navigation.Filters.IntegratedNavSoln.stdev(:,7), 'r')
    plot(DATA.t, -3*DATA.FC.Navigation.Filters.IntegratedNavSoln.stdev(:,7), 'r')
    title('Nav Solution X Position Error')

    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, DATA.state.yE - DATA.FC.Navigation.Filters.InertialNavSoln.rhat(:,2))
    plot(DATA.t, 3*DATA.FC.Navigation.Filters.IntegratedNavSoln.stdev(:,8), 'r')
    plot(DATA.t, -3*DATA.FC.Navigation.Filters.IntegratedNavSoln.stdev(:,8), 'r')
    title('Nav Solution Y Position Error')

    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, DATA.state.zE - DATA.FC.Navigation.Filters.InertialNavSoln.rhat(:,3))
    plot(DATA.t, 3*DATA.FC.Navigation.Filters.IntegratedNavSoln.stdev(:,9), 'r')
    plot(DATA.t, -3*DATA.FC.Navigation.Filters.IntegratedNavSoln.stdev(:,9), 'r')
    title('Nav Solution Z Position Error')


    % Nav Solution Velocity Errors
    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, DATA.state.vX - DATA.FC.Navigation.Filters.InertialNavSoln.vhat(:,1))
    plot(DATA.t, 3*DATA.FC.Navigation.Filters.IntegratedNavSoln.stdev(:,4), 'r')
    plot(DATA.t, -3*DATA.FC.Navigation.Filters.IntegratedNavSoln.stdev(:,4), 'r')
    title('Nav Solution X Velocity Error')

    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, DATA.state.vY - DATA.FC.Navigation.Filters.InertialNavSoln.vhat(:,2))
    plot(DATA.t, 3*DATA.FC.Navigation.Filters.IntegratedNavSoln.stdev(:,5), 'r')
    plot(DATA.t, -3*DATA.FC.Navigation.Filters.IntegratedNavSoln.stdev(:,5), 'r')
    title('Nav Solution Y Velocity Error')

    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, DATA.state.vZ - DATA.FC.Navigation.Filters.InertialNavSoln.vhat(:,3))
    plot(DATA.t, 3*DATA.FC.Navigation.Filters.IntegratedNavSoln.stdev(:,6), 'r')
    plot(DATA.t, -3*DATA.FC.Navigation.Filters.IntegratedNavSoln.stdev(:,6), 'r')
    title('Nav Solution Z Velocity Error')


    % Nav Solution Attitude Errors
    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, (DATA.state.phi - DATA.FC.Navigation.Filters.InertialNavSoln.rpyhat(:,1)) .* 180/pi)
    plot(DATA.t, 3*DATA.FC.Navigation.Filters.IntegratedNavSoln.stdev(:,1) .* 180/pi, 'r')
    plot(DATA.t, -3*DATA.FC.Navigation.Filters.IntegratedNavSoln.stdev(:,1) .* 180/pi, 'r')
    title('Nav Solution Bank Angle Error')

    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, (DATA.state.theta - DATA.FC.Navigation.Filters.InertialNavSoln.rpyhat(:,2)) .* 180/pi)
    plot(DATA.t, 3*DATA.FC.Navigation.Filters.IntegratedNavSoln.stdev(:,2) .* 180/pi, 'r')
    plot(DATA.t, -3*DATA.FC.Navigation.Filters.IntegratedNavSoln.stdev(:,2) .* 180/pi, 'r')
    title('Nav Solution Pitch Angle Error')

    figure
    hold on
    grid on
    set(gcf, 'Color', 'w');
    set(gcf, 'Position', plotPos)

    plot(DATA.t, (DATA.state.psi - unwrap(DATA.FC.Navigation.Filters.InertialNavSoln.rpyhat(:,3))) .* 180/pi)
    plot(DATA.t, 3*DATA.FC.Navigation.Filters.IntegratedNavSoln.stdev(:,3) .* 180/pi, 'r')
    plot(DATA.t, -3*DATA.FC.Navigation.Filters.IntegratedNavSoln.stdev(:,3) .* 180/pi, 'r')
    title('Nav Solution Yaw Angle Error')


end