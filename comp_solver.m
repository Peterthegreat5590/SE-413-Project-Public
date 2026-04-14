function [score, ineq] = comp_solver(front_aero_frontal_area, rear_aero_frontal_area, vehicle)
        % Update vehicle object fields with new values for aero frontal area
    % and weight.
    vehicle.FrontAeroFrontalArea = front_aero_frontal_area;
    vehicle.RearAeroFrontalArea = rear_aero_frontal_area;
    vehicle = vehicle_model(vehicle);

    accel_time = accel_event_solve(vehicle);
    t_min_accel = min(3.606,accel_time);
    t_max_accel = 1.5*t_min_accel; 
    accel_time_bounded = min(t_max_accel,accel_time);
    accel_score = 95.5*((t_max_accel/accel_time_bounded-1)/(t_max_accel/t_min_accel-1)) + 4.5;
    
    [skidpad_time, alpha, velo_max] = skidpad_event_solve(vehicle);
    t_min_skidpad = min(4.65,skidpad_time);
    t_max_skidpad = 1.25*t_min_skidpad;
    skidpad_time_bounded = min(t_max_skidpad,skidpad_time);
    skidpad_score = 71.5*(((t_max_skidpad/skidpad_time_bounded)^2 - 1)/((t_max_skidpad/t_min_skidpad)^2 - 1)) + 3.5;

    % disp([skidpad_score, accel_score])
    % disp([skidpad_time, accel_time])
    
    score = skidpad_score + accel_score;

    % Minimum Critical Velocity for limit oversteer in m/s (Top Gear Top
    % Speed)
    vcrit_min = 34.0894;

    ay = velo_max^2/8.325; % v^2/r
    alpha_f = alpha(1);
    alpha_r = alpha(2);
    k = (alpha_f-alpha_r)/ay;
    % disp(alpha)
    if k<0
        vcrit = sqrt(-vehicle.Wheelbase/k);
    else
        vcrit = 1000;
    end
    ineq = vcrit_min - vcrit;
end