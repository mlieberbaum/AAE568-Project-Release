function outputStateVector = sixDofRigidAircraftSolverRK45(dt, state, netForceVector, netMomentVector, spaceplane)

    % For readability
    u = state.u;
    v = state.v;
    w = state.w;
    p = state.p;
    q = state.q;
    r = state.r;
    phi = state.phi;
    theta = state.theta;
    psi = state.psi;
    xE = state.xE;
    yE = state.yE;
    zE = state.zE;
    X = netForceVector(1);
    Y = netForceVector(2);
    Z = netForceVector(3);
    L = netMomentVector(1);
    M = netMomentVector(2);
    N = netMomentVector(3);
    
    mass = spaceplane.mass;
    Ixx = spaceplane.Ixx;
    Iyy = spaceplane.Iyy;
    Izz = spaceplane.Izz;
    Ixy = spaceplane.Ixy;
    Ixz = spaceplane.Ixz;
    Iyz = spaceplane.Iyz;
    
    
    % RK45
    x0 = [u; v; w; xE; yE; zE; p; q; r; phi; theta; psi];
    t0 = 0;
    tf = dt;
    
    fdot = @(t,x) [ (X / mass) - (x(8) * x(3)) + (x(9) * x(2));
                    (Y / mass) - (x(9) * x(1)) + (x(7) * x(3));
                    (Z / mass) - (x(7) * x(2)) + (x(8) * x(1))
                    x(1) * ((cos(x(11)) * cos(x(12)))) + x(2) * ((sin(x(10)) * sin(x(11)) * cos(x(12))) - (cos(x(10)) * sin(x(12)))) + x(3) * ((cos(x(10)) * sin(x(11)) * cos(x(12))) + (sin(x(10)) * sin(x(12))));
                    x(1) * ((cos(x(11)) * sin(x(12)))) + x(2) * ((sin(x(10)) * sin(x(11)) * sin(x(12))) + (cos(x(10)) * cos(x(12)))) + x(3) * ((cos(x(10)) * sin(x(11)) * sin(x(12))) - (sin(x(10)) * cos(x(12))));
                   -x(1) * ((sin(x(11))))              + x(2) * ((sin(x(10)) * cos(x(11))))                                          + x(3) * ((cos(x(10)) * cos(x(11))));
                    ((Iyz^2 - Iyy*Izz)*(L - x(8)*(Ixz*x(7) + Iyz*x(8) + Izz*x(9)) + x(9)*(Ixy*x(7) + Iyy*x(8) + Iyz*x(9))))/(Izz*Ixy^2 - 2*Ixy*Ixz*Iyz + Iyy*Ixz^2 + Ixx*Iyz^2 - Ixx*Iyy*Izz) - ((Ixy*Iyz - Ixz*Iyy)*(N - x(7)*(Ixy*x(7) + Iyy*x(8) + Iyz*x(9)) + x(8)*(Ixx*x(7) + Ixy*x(8) + Ixz*x(9))))/(Izz*Ixy^2 - 2*Ixy*Ixz*Iyz + Iyy*Ixz^2 + Ixx*Iyz^2 - Ixx*Iyy*Izz) - ((Ixz*Iyz - Ixy*Izz)*(M + x(7)*(Ixz*x(7) + Iyz*x(8) + Izz*x(9)) - x(9)*(Ixx*x(7) + Ixy*x(8) + Ixz*x(9))))/(Izz*Ixy^2 - 2*Ixy*Ixz*Iyz + Iyy*Ixz^2 + Ixx*Iyz^2 - Ixx*Iyy*Izz)
                    ((Ixz^2 - Ixx*Izz)*(M + x(7)*(Ixz*x(7) + Iyz*x(8) + Izz*x(9)) - x(9)*(Ixx*x(7) + Ixy*x(8) + Ixz*x(9))))/(Izz*Ixy^2 - 2*Ixy*Ixz*Iyz + Iyy*Ixz^2 + Ixx*Iyz^2 - Ixx*Iyy*Izz) - ((Ixy*Ixz - Ixx*Iyz)*(N - x(7)*(Ixy*x(7) + Iyy*x(8) + Iyz*x(9)) + x(8)*(Ixx*x(7) + Ixy*x(8) + Ixz*x(9))))/(Izz*Ixy^2 - 2*Ixy*Ixz*Iyz + Iyy*Ixz^2 + Ixx*Iyz^2 - Ixx*Iyy*Izz) - ((Ixz*Iyz - Ixy*Izz)*(L - x(8)*(Ixz*x(7) + Iyz*x(8) + Izz*x(9)) + x(9)*(Ixy*x(7) + Iyy*x(8) + Iyz*x(9))))/(Izz*Ixy^2 - 2*Ixy*Ixz*Iyz + Iyy*Ixz^2 + Ixx*Iyz^2 - Ixx*Iyy*Izz)
                    ((Ixy^2 - Ixx*Iyy)*(N - x(7)*(Ixy*x(7) + Iyy*x(8) + Iyz*x(9)) + x(8)*(Ixx*x(7) + Ixy*x(8) + Ixz*x(9))))/(Izz*Ixy^2 - 2*Ixy*Ixz*Iyz + Iyy*Ixz^2 + Ixx*Iyz^2 - Ixx*Iyy*Izz) - ((Ixy*Ixz - Ixx*Iyz)*(M + x(7)*(Ixz*x(7) + Iyz*x(8) + Izz*x(9)) - x(9)*(Ixx*x(7) + Ixy*x(8) + Ixz*x(9))))/(Izz*Ixy^2 - 2*Ixy*Ixz*Iyz + Iyy*Ixz^2 + Ixx*Iyz^2 - Ixx*Iyy*Izz) - ((Ixy*Iyz - Ixz*Iyy)*(L - x(8)*(Ixz*x(7) + Iyz*x(8) + Izz*x(9)) + x(9)*(Ixy*x(7) + Iyy*x(8) + Iyz*x(9))))/(Izz*Ixy^2 - 2*Ixy*Ixz*Iyz + Iyy*Ixz^2 + Ixx*Iyz^2 - Ixx*Iyy*Izz)
                    x(7) + (x(8) * sin(x(10)) * tan(x(11))) + (x(9) * cos(x(10)) * tan(x(11)));
                    (x(8) * cos(x(10))) - (x(9) * sin(x(10)));
                    (x(8) * sin(x(10)) * sec(x(11))) + (x(9) * cos(x(10)) * sec(x(11)));
                ];
    
    S = odeset('RelTol', 1e-8, 'AbsTol', 1e-8);
    [tout, yout] = ode45(fdot, [t0 tf], x0, S);
    
    
    % Make sure tout is valid
    if abs(tout(end) - dt) > eps
        error('Error: Invalid RK45 Propagation!  End time not correct.');
    end
    
    
    % Output state
    outputStateVector.u = yout(end,1);
    outputStateVector.v = yout(end,2);
    outputStateVector.w = yout(end,3);
    outputStateVector.xE = yout(end,4);
    outputStateVector.yE = yout(end,5);
    outputStateVector.zE = yout(end,6);
    outputStateVector.p = yout(end,7);
    outputStateVector.q = yout(end,8);
    outputStateVector.r = yout(end,9);
    outputStateVector.phi = yout(end,10);
    outputStateVector.theta = yout(end,11);
    outputStateVector.psi = yout(end,12);
    outputStateVector.vX = (yout(end, 4) - yout(1, 4)) / dt;
    outputStateVector.vY = (yout(end, 5) - yout(1, 5)) / dt;
    outputStateVector.vZ = (yout(end, 6) - yout(1, 6)) / dt;
    
    
end