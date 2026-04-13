clear;
clc;


%% Constants
vehicle = struct;
vehicle.BareMass = 175.6; % Mass in Kg without aero package
vehicle.CLF = -3.92; % Coefficient of Lift for Front Aero Surface
vehicle.CLR = -4.44; % Coefficient of Lift for Rear Aero Surface
vehicle.CDF = 0.845; % Coefficient of Drag for Front Aero Surface
vehicle.CDR = 2.315; % Coefficient of Drag for Rear Aero Surface
vehicle.hDF = 0.245; % Height of center of drag for Front Aero Surface
vehicle.hDR = 0.877; % Height of center of drag for Rear Aero Surface
vehicle.lLF = 2.167; % Distance from center of lift to Rear Tire for Front Aero Surface
vehicle.lLR = 0.299; % Distance from center of lift to Rear Tire for Rear Aero Surface
vehicle.Torque = 542; % Maximum Wheel Torque From Powertrain in Nm
vehicle.Power = 61; % Maximum powertrain power in KW
vehicle.Wheelbase = 1.524; % Wheelbase in m
vehicle.Trackwidth = 1.2446; % Trackwidth in m
vehicle.FrontAeroWeightNorm = 12.726; % Weight of Front Aero Package per square meter of frontal area
vehicle.RearAeroWeightNorm = 18.673; % Weight of Rear Aero Package per square meter of frontal area
vehicle.TireRadius = 0.2032; % 16 inch diameter tire radius in meters
vehicle.CoGHeight = 0.244348; % Height of Center of Gravity
vehicle.CoGFromRT = 0.70104; % Distance of Center of Gravity from Rear Tire
vehicle.CoPHeight = 0.6871; % Height of Center of Pressure (Total Aero Effect)
vehicle.CoPFromRT = 0.9577; % Distance of Center of Gravity from Rear Tire


%% Variables
vehicle.FrontAeroFrontalArea = 0.5204; % Front Frontal area in m^2, default value
vehicle.RearAeroFrontalArea = 0.4421; % Rear Frontal area in m^2, default value
vehicle.Mass = 190.5; % Weight of




function score = objective_function(front_aero_frontal_area, rear_aero_frontal_area, vehicle)
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
    
    skidpad_time = skidpad_event_solve(vehicle);
    t_min_skidpad = min(4.65,skidpad_time);
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


areas = 0:0.01:1.4;
scores = zeros(size(areas));

sart_time = datetime;
for i = 1:length(areas)
    scores(i) = objective_function(areas(i),vehicle);
end
total_time = datetime-start_time;
total_time.Format = 'mm:ss.SSS';
disp(total_time)

plot(areas,scores);
xlabel("Aerodynamic Frontal Area (m^2)")
ylabel("Total Score")
title("Total Score vs. Aerodynamic Frontal Area")