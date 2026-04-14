function fy = slip_angle_tire_model(fz, alpha)
    % SLIP_ANGLE_TIRE_MODEL Finds the lateral force from a specific slip
    % angle and normal force using Pajecka's Magic Formula Model
    %   Fy = SLIP_ANGLE_TIRE_MODEL(Fz, Alpha)
    
    % Fitting coefficients from data (see Tire_Fitting.py for more info)
    B = 11.32;
    C = 1.454;
    D = 2.435;

    fy = fz*D*sin(C*atan(B*alpha));

end