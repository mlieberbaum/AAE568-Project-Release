function ctrlSurf = commandControlSurf(FC)

    ctrlSurf.elevator = bound(FC.ControlSurfCommand.elevator, -1, 1);
    ctrlSurf.aileron = bound(FC.ControlSurfCommand.aileron, -1, 1);
    ctrlSurf.rudder = bound(FC.ControlSurfCommand.rudder, -1, 1);
    ctrlSurf.gear = FC.ControlSurfCommand.gear;
    ctrlSurf.trim = bound(FC.ControlSurfCommand.trim, -1, 1);
    ctrlSurf.spdbk = bound(FC.ControlSurfCommand.spdbk, -1, 1);
    
end