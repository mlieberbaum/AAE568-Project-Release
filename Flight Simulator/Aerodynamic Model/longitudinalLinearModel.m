function [A, B, Actl, Bctl, deriv] = longitudinalLinearModel(state, altitude, spacecraft, rho, mach, ctrlSurf)

    % Function to return the longitudinal linearized model of an
    % airframe about a steady flight condition
    
    d2r = pi / 180;
    wind = [0 0 0];
    
    % Reference pitch angle
    th0 = state.theta;
    u0 = state.u;
    
    
    % Perturbation magnitudes
    del_u = 0.1;
    del_w = 0.1;
    del_q = 0.1 * d2r;
    del_de = 0.05;
    
    
    % Perturbations
    state_uPlus = state;
    state_uPlus.u = state_uPlus.u + del_u;
    
    state_uMinus = state;
    state_uMinus.u = state_uMinus.u - del_u;
    
    state_wPlus = state;
    state_wPlus.w = state_wPlus.w + del_w;
    
    state_wMinus = state;
    state_wMinus.w = state_wMinus.w - del_w;
    
    state_qPlus = state;
    state_qPlus.q = state_qPlus.q + del_q;
    
    state_qMinus = state;
    state_qMinus.q = state_qMinus.q - del_q;
    
    ctrlSurf_dePlus = ctrlSurf;
    ctrlSurf_dePlus.elevator = ctrlSurf_dePlus.elevator + del_de;
    
    ctrlSurf_deMinus = ctrlSurf;
    ctrlSurf_deMinus.elevator = ctrlSurf_deMinus.elevator - del_de;
    
    
    % Calculate g
    g = 398600439968871 / ((altitude + 6371010.0)^2);
    
    
    % Deltas from Perturbations
    [netForceVecInB_uPlus, ~, ~, ~, ~, netMomentVecInB_uPlus] = aerodynamicModel(state_uPlus, altitude, spacecraft, rho, mach, ctrlSurf, wind);
    [netForceVecInB_uMinus, ~, ~, ~, ~, netMomentVecInB_uMinus] = aerodynamicModel(state_uMinus, altitude, spacecraft, rho, mach, ctrlSurf, wind);
    
    [netForceVecInB_wPlus, ~, ~, ~, ~, netMomentVecInB_wPlus] = aerodynamicModel(state_wPlus, altitude, spacecraft, rho, mach, ctrlSurf, wind);
    [netForceVecInB_wMinus, ~, ~, ~, ~, netMomentVecInB_wMinus] = aerodynamicModel(state_wMinus, altitude, spacecraft, rho, mach, ctrlSurf, wind);
    
    [~, ~, ~, ~, ~, netMomentVecInB_qPlus] = aerodynamicModel(state_qPlus, altitude, spacecraft, rho, mach, ctrlSurf, wind);
    [~, ~, ~, ~, ~, netMomentVecInB_qMinus] = aerodynamicModel(state_qMinus, altitude, spacecraft, rho, mach, ctrlSurf, wind);
    
    [netForceVecInB_dePlus, ~, ~, ~, ~, netMomentVecInB_dePlus] = aerodynamicModel(state, altitude, spacecraft, rho, mach, ctrlSurf_dePlus, wind);
    [netForceVecInB_deMinus, ~, ~, ~, ~, netMomentVecInB_deMinus] = aerodynamicModel(state, altitude, spacecraft, rho, mach, ctrlSurf_deMinus, wind);
    
    
    % Stability Derivatives
    du = 2 * del_u;
    dw = 2 * del_w;
    dq = 2 * del_q;
    dde = 2 * del_de;
    
    Xu =  (netForceVecInB_uPlus(1)   - netForceVecInB_uMinus(1))   / du;
    Xw =  (netForceVecInB_wPlus(1)   - netForceVecInB_wMinus(1))   / dw;
    Zu =  (netForceVecInB_uPlus(3)   - netForceVecInB_uMinus(3))   / du;
    Zw =  (netForceVecInB_wPlus(3)   - netForceVecInB_wMinus(3))   / dw;
    Mu =  (netMomentVecInB_uPlus(2)  - netMomentVecInB_uMinus(2))  / du;
    Mw =  (netMomentVecInB_wPlus(2)  - netMomentVecInB_wMinus(2))  / dw;
    Mq =  (netMomentVecInB_qPlus(2)  - netMomentVecInB_qMinus(2))  / dq;
    Xde = (netForceVecInB_dePlus(1)  - netForceVecInB_deMinus(1))  / dde;
    Zde = (netForceVecInB_dePlus(3)  - netForceVecInB_deMinus(3))  / dde;
    Mde = (netMomentVecInB_dePlus(2) - netMomentVecInB_deMinus(2)) / dde;
    
    
    % Approximations (From Nelson p. 165)
    Zalpha = u0 * Zw;
    Malpha = u0 * Mw;
    
        
    % Linearized matrices (u, w, q, theta)
    m = spacecraft.mass;
    Iyy = spacecraft.Iyy;
    
    A = [ Xu/m     Xw/m     0       -g*cos(th0)
          Zu/m     Zw/m     u0      -g*sin(th0)
          Mu/Iyy   Mw/Iyy   Mq/Iyy   0
          0        0        1        0             ];
      
    B = [ Xde/m
          Zde/m
          Mde/Iyy
          0       ];

    
    % Linearized matrices for controller design (alpha, q, theta, gamma)
    Actl = [ Zalpha/u0/m    1        0     0
             Malpha/Iyy     Mq/Iyy   0     0
             0              1        0     0
            -1              0        1     0];

    Bctl = [Zde/u0 /m
            Mde/Iyy
            0
            0        ];
      
      
    % Stability Derivatives
    deriv.Xu = Xu;
    deriv.Xw = Xw;
    deriv.Zu = Zu;
    deriv.Zw = Zw;
    deriv.Mu = Mu;
    deriv.Mw = Mw;
    deriv.Mq = Mq;
    deriv.Xde = Xde;
    deriv.Zde = Zde;
    deriv.Mde = Mde;
    deriv.Zalpha = Zalpha;
    deriv.Malpha = Malpha;
    deriv.u0 = u0;
    deriv.th0 = th0;
    
end