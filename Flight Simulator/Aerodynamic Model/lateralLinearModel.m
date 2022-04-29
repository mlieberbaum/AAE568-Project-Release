function [A, B, Actl, Bctl, deriv] = lateralLinearModel(state, altitude, spacecraft, rho, mach, ctrlSurf)

    % Function to return the lateral linearized model of an airframe about 
    % a steady flight condition
    
    d2r = pi / 180;
    wind = [0 0 0];
    
    % Reference pitch angle
    th0 = state.theta;
    u0 = state.u;
    
    
    % Perturbation magnitudes
    del_v = 0.1;
    del_p = 0.1 * d2r;
    del_r = 0.1 * d2r;
    del_dr = 0.05;
    del_da = 0.05;
    
    
    % Perturbations
    state_vPlus = state;
    state_vPlus.v = state_vPlus.v + del_v;
    
    state_vMinus = state;
    state_vMinus.v = state_vMinus.v - del_v;
    
    state_pPlus = state;
    state_pPlus.p = state_pPlus.p + del_p;
    
    state_pMinus = state;
    state_pMinus.p = state_pMinus.p - del_p;
    
    state_rPlus = state;
    state_rPlus.r = state_rPlus.r + del_r;
    
    state_rMinus = state;
    state_rMinus.r = state_rMinus.r - del_r;
    
    ctrlSurf_drPlus = ctrlSurf;
    ctrlSurf_drPlus.rudder = ctrlSurf_drPlus.rudder + del_dr;
    
    ctrlSurf_drMinus = ctrlSurf;
    ctrlSurf_drMinus.rudder = ctrlSurf_drMinus.rudder - del_dr;
    
    ctrlSurf_daPlus = ctrlSurf;
    ctrlSurf_daPlus.aileron = ctrlSurf_daPlus.aileron + del_da;
    
    ctrlSurf_daMinus = ctrlSurf;
    ctrlSurf_daMinus.aileron = ctrlSurf_daMinus.aileron - del_da;
    
    
    % Calculate g
    g = 398600439968871 / ((altitude + 6371010.0)^2);
    
    
    % Deltas from Perturbations
    [netForceVecInB_vPlus, ~, ~, ~, ~, netMomentVecInB_vPlus] = aerodynamicModel(state_vPlus, altitude, spacecraft, rho, mach, ctrlSurf, wind);
    [netForceVecInB_vMinus, ~, ~, ~, ~, netMomentVecInB_vMinus] = aerodynamicModel(state_vMinus, altitude, spacecraft, rho, mach, ctrlSurf, wind);
    
    [netForceVecInB_pPlus, ~, ~, ~, ~, netMomentVecInB_pPlus] = aerodynamicModel(state_pPlus, altitude, spacecraft, rho, mach, ctrlSurf, wind);
    [netForceVecInB_pMinus, ~, ~, ~, ~, netMomentVecInB_pMinus] = aerodynamicModel(state_pMinus, altitude, spacecraft, rho, mach, ctrlSurf, wind);
    
    [~, ~, ~, ~, ~, netMomentVecInB_rPlus] = aerodynamicModel(state_rPlus, altitude, spacecraft, rho, mach, ctrlSurf, wind);
    [~, ~, ~, ~, ~, netMomentVecInB_rMinus] = aerodynamicModel(state_rMinus, altitude, spacecraft, rho, mach, ctrlSurf, wind);
    
    [netForceVecInB_drPlus, ~, ~, ~, ~, netMomentVecInB_drPlus] = aerodynamicModel(state, altitude, spacecraft, rho, mach, ctrlSurf_drPlus, wind);
    [netForceVecInB_drMinus, ~, ~, ~, ~, netMomentVecInB_drMinus] = aerodynamicModel(state, altitude, spacecraft, rho, mach, ctrlSurf_drMinus, wind);
    
    [~, ~, ~, ~, ~, netMomentVecInB_daPlus] = aerodynamicModel(state, altitude, spacecraft, rho, mach, ctrlSurf_daPlus, wind);
    [~, ~, ~, ~, ~, netMomentVecInB_daMinus] = aerodynamicModel(state, altitude, spacecraft, rho, mach, ctrlSurf_daMinus, wind);
    
    
    % Stability Derivatives
    dv = 2 * del_v;
    dp = 2 * del_p;
    dr = 2 * del_r;
    ddr = 2 * del_dr;
    dda = 2 * del_da;
    
    Yv =  (netForceVecInB_vPlus(2)   - netForceVecInB_vMinus(2))   / dv;
    Yp =  (netForceVecInB_pPlus(2)   - netForceVecInB_pMinus(2))   / dp;
    Lv =  (netMomentVecInB_vPlus(1)  - netMomentVecInB_vMinus(1))  / dv;
    Lp =  (netMomentVecInB_pPlus(1)  - netMomentVecInB_pMinus(1))  / dp;
    Lr =  (netMomentVecInB_rPlus(1)  - netMomentVecInB_rMinus(1))  / dr;
    Nv =  (netMomentVecInB_vPlus(3)  - netMomentVecInB_vMinus(3))  / dv;
    Np =  (netMomentVecInB_pPlus(3)  - netMomentVecInB_pMinus(3))  / dp;
    Nr =  (netMomentVecInB_rPlus(3)  - netMomentVecInB_rMinus(3))  / dr;
    Ydr = (netForceVecInB_drPlus(2)  - netForceVecInB_drMinus(2))  / ddr;
    Lda = (netMomentVecInB_daPlus(1) - netMomentVecInB_daMinus(1)) / dda;
    Ldr = (netMomentVecInB_drPlus(1) - netMomentVecInB_drMinus(1)) / ddr;
    Nda = (netMomentVecInB_daPlus(3) - netMomentVecInB_daMinus(3)) / dda;
    Ndr = (netMomentVecInB_drPlus(3) - netMomentVecInB_drMinus(3)) / ddr;
    
    
    % Linearized matrices (v, p, r, phi)
    m = spacecraft.mass;
    Ixx = spacecraft.Ixx;
    Izz = spacecraft.Izz;
    
    A = [ Yv/m     Yp/m    -u0           g*cos(th0)
          Lv/Ixx   Lp/Ixx   Lr/Ixx       0         
          Nv/Izz   Np/Izz   Nr/Izz       0         
          0        1        tan(th0)     0         ];
      
    B = [ 0         Ydr/m
          Lda/Ixx   Ldr/Ixx
          Nda/Izz   Ndr/Izz
          0         0       ];
    
    
    % From Nelson, page 205
    Yb = Yv * u0;
    Lb = Lv * u0;
    Nb = Nv * u0;
    
    
    % Linearized matrices for control (beta, p, r, phi)
    Actl = [ Yb/u0/m     Yp/u0/m    -1            g*cos(th0)/u0
             Lb/Ixx      Lp/Ixx      Lr/Ixx       0         
             Nb/Izz      Np/Izz      Nr/Izz       0         
             0           1           tan(th0)     0         ];
    
    Bctl = [ 0         Ydr/m/u0
             Lda/Ixx   Ldr/Ixx
             Nda/Izz   Ndr/Izz
             0         0       ];
      
      
    % Stability Derivatives
    deriv.Yv = Yv;
    deriv.Yp = Yp;
    deriv.Lv = Lv;
    deriv.Lp = Lp;
    deriv.Lr = Lr;
    deriv.Nv = Nv;
    deriv.Np = Np;
    deriv.Nr = Nr;
    deriv.Ydr = Ydr;
    deriv.Lda = Lda;
    deriv.Ldr = Ldr;
    deriv.Nda = Nda;
    deriv.Ndr = Ndr;
    
end



