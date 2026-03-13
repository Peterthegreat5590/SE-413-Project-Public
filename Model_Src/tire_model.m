function [f_lateral, f_longitudinal] = tire_model(f_normal)
    f_lateral_norm = 1604.465; % Fy reference in N
    f_long_norm = 1467.816; % Fx reference in N
    f_norm = 667; % Fz at Fx anf Fy reference in N
    scaling_factor = f_normal/f_norm; % Linear scaling based on tire data regression, reduced by 
    f_lateral = f_lateral_norm*scaling_factor;
    f_longitudinal = f_long_norm*scaling_factor;
end