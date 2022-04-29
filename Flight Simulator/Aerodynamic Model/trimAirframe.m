function tr = trimAirframe(spacecraft, aspd, altitude, rho, mach)


    % Trim Condition
    tr = struct();
    

    % Assume no control surfaces for trimmed condition
    ctrlSurf = struct();
    ctrlSurf.gear = 0;
    ctrlSurf.spdbk = 0;
    ctrlSurf.elevator = 0;
    ctrlSurf.aileron = 0;
    ctrlSurf.rudder = 0;
    ctrlSurf.trim = 0;
    
    
    % For the trim conditions, we require steady level flight, i.e. Lift =
    % Weight.
    qinf = 0.5 * rho * aspd * aspd;
    L = spacecraft.mass * 9.81;
    cL = L / (qinf * spacecraft.mainWing.S);
    alpha = interp1(spacecraft.mainWing.cLvsAlpha(:,2), spacecraft.mainWing.cLvsAlpha(:,1), cL);
    
    
    % Build the state
    state.u = aspd * cos(alpha);
    state.v = 0;
    state.w = aspd * sin(alpha);
    state.p = 0;
    state.q = 0;
    state.r = 0;
    state.psi = 0;
    state.theta = alpha;    % Because gamma=0 for steady level flight
    state.phi = 0;
    state.xE = 0;
    state.yE = 0;
    state.zE = -altitude;
    
    
    % Longitudinal linearized model
    [tr.lon.A, tr.lon.B, tr.lon.Actl, tr.lon.Bctl, tr.lon.deriv] = longitudinalLinearModel(state, altitude, spacecraft, rho, mach, ctrlSurf);
    
    % Lateral linearized model
    [tr.lat.A, tr.lat.B, tr.lat.Actl, tr.lat.Bctl, tr.lat.deriv] = lateralLinearModel(state, altitude, spacecraft, rho, mach, ctrlSurf);
    
    % Pitch angle for trim point
    tr.theta0 = state.theta;
    
    
end