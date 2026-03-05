

%% Constants
vehicle.BareMass = 175.6; % Mass in Kg without aero package
vehicle.CL = -4.06; % Coefficient of Lift
vehicle.CD = 1.58; % Coefficient of Drag
vehicle.Torque = 542; % Maximum Wheel Torque From Powertrain in Nm
vehicle.Power = 61; % Maximum powertrain power in KW
vehicle.Wheelbase = 1.524; % Wheelbase in m
vehicle.Trackwidth = 1.2446; % Trackwidth in m
vehicle.AeroWeightNorm = 15.47; % Weight of Aero Package per square meter of frontal area
vehicle.TireRadius = 0.2032; % 16 inch diameter tire radius in meters
vehicle.CoGHeight = 0.244348; % Height of Center of Gravity
vehicle.CoGFromRT = 0.70104; % Distance of Center of Gravity from Rear Tire


%% Variables
vehicle.AeroFrontalArea = 0.95; % Frontal area in m^2, default value
vehicle.Mass = 190.5; % Weight of



function score = objective_function(aero_frontal_area, vehicle)
    % Update vehicle object fields with new values for aero frontal area
    % and weight.
    vehicle.AeroFrontalArea = aero_frontal_area;
    vehicle = vehicle_model(vehicle);

    accel_time = accel_event_solve(vehicle);
    t_min_accel = min(4.169,accel_time);
    t_max_accel = 1.5*t_min_accel; 
    accel_time_bounded = min(t_max_accel,accel_time);
    accel_score = 95.5*((t_max_accel/accel_time_bounded-1)/(t_max_accel/t_min_accel-1)) + 4.5;
    
    skidpad_time = skidpad_event_solve(vehicle);
    t_min_skidpad = min(5.080,skidpad_time);
    t_max_skidpad = 1.25*t_min_skidpad;
    skidpad_time_bounded = min(t_max_skidpad,skidpad_time);
    skidpad_score = 71.5*(((t_max_skidpad/skidpad_time_bounded)^2 - 1)/((t_max_skidpad/t_min_skidpad)^2 - 1)) + 3.5;

    disp([skidpad_score, accel_score])
    disp([skidpad_time, accel_time])
    
    score = skidpad_score + accel_score;
end

start_time = datetime;
score = objective_function(0.9, vehicle);
total_time = datetime - start_time;
total_time.Format = 'mm:ss.SSS';
disp(score)
disp(total_time)