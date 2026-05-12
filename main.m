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


% Example for calling solver directly without minimizer:
% start_time = datetime;
% score = comp_solver(0.4,0.5, vehicle);
% total_time = datetime - start_time;
% total_time.Format = 'mm:ss.SSS';
% disp(score)
% disp(total_time)

start_time = datetime;
[x,f,flag,outp] = edo_problem_solver(vehicle);
xopt = x;
front_opt = xopt(1);
rear_opt  = xopt(2);
total_time = datetime - start_time;
total_time.Format = 'mm:ss.SSS';
disp(total_time)

disp(-f)
disp(x)

front = 0:0.1:1.5;
rear = 0:0.1:1.5;

[Fr, Re] = meshgrid(front, rear);

Ob = zeros(size(Fr));

%optimal vehicle model
vehicleOpt = vehicle;
AccelTime = zeros(size(Fr));
SkidpadTime = zeros(size(Fr));

%solver opt score
[score_opt, constraint_opt] = comp_solver(front_opt, rear_opt, vehicle);
accel_time = accel_event_solve(vehicleOpt);
[skidpad_time, ~, ~] = skidpad_event_solve(vehicleOpt);

for i = 1:length(rear)
    for j = 1:length(front)
        %disp(i);
        %disp(j);
        [score, ineq] = comp_solver(Fr(i,j),Re(i,j),vehicle);
        Ob(i,j) = score;

        vehicle_temp = vehicle;
        vehicle_temp.FrontAeroFrontalArea = Fr(i,j);
        vehicle_temp.RearAeroFrontalArea = Re(i,j);
        vehicle_temp = vehicle_model(vehicle_temp);

        AccelTime(i,j) = accel_event_solve(vehicle_temp);
        [SkidpadTime(i,j), ~, ~] = skidpad_event_solve(vehicle_temp);
    end
end

%score vs surface area
hold on
contourf(Fr, Re, Ob, 21)
plot(front_opt, rear_opt, "k.",MarkerSize=20)
c = colorbar();
c.Label.String = "Total Score";
xlabel("Front Aerodynamic Frontal Area (m^2)")
ylabel("Rear Aerodynamic Frontal Area (m^2)")
title("Predicted Score of a FSAE car vs Aerodynamic Surface Area")
legend("Location", "best");
grid on;
box on;

%trade off plots
figure;
contourf(Fr, Re, AccelTime, 21, "LineColor", "none");
hold on;

plot(front_opt, rear_opt, "kp", "MarkerSize", 14, "MarkerFaceColor", "y");

c = colorbar();
c.Label.String = "Acceleration Time (s)";

xlabel("Front Aerodynamic Frontal Area (m^2)");
ylabel("Rear Aerodynamic Frontal Area (m^2)");
title("Acceleration Time vs. Front and Rear Aero Area");

grid on;
box on;

%Skidpad Time Surface

figure;
contourf(Fr, Re, SkidpadTime, 21, "LineColor", "none");
hold on;

plot(front_opt, rear_opt, "kp", "MarkerSize", 14, "MarkerFaceColor", "y");

c = colorbar();
c.Label.String = "Skidpad Time [s]";

xlabel("Front Aerodynamic Frontal Area [m^2]");
ylabel("Rear Aerodynamic Frontal Area [m^2]");
title("Skidpad Time vs. Front and Rear Aero Area");

grid on;
box on;

%multistart
num_start_points = 8;
use_parralel = true;

if isempty(gcp("nocreate"))
    parpool;
end

start_time_ms = datetime;
[x_ms, f_ms, exitflag_ms, output_ms, solutions_ms] = multistart_solver(vehicle, num_start_points, use_parralel);
total_time_ms = datetime - start_time_ms;
total_time_ms.Format = 'mm:ss.SSS';

score_ms = -f_ms;

%results
fprintf("\nOPTIMIZATION SUMMARY:\n");
fprintf("Solver exit flag: %d\n", flag);
fprintf("Solver message: %s\n", outp.message);
fprintf("Iterations: %d\n", outp.iterations);
fprintf("Function evaluations: %d\n", outp.funcCount);
fprintf("Runtime: %s\n", string(total_time));

fprintf("\nOptimal Design Variables:\n");
fprintf("Front aero frontal area = %.4f m^2\n", front_opt);
fprintf("Rear aero frontal area  = %.4f m^2\n", rear_opt);
fprintf("Total aero frontal area = %.4f m^2\n", front_opt + rear_opt);

fprintf("\nPerformance:\n");
fprintf("Total score = %.4f points\n", score_opt);
fprintf("Acceleration time = %.4f s\n", accel_time);
fprintf("Skidpad time = %.4f s\n", skidpad_time);

fprintf("\nMULTISTART RESULTS: \n");
fprintf("Runtime: %s\n", string(total_time_ms));
fprintf("Best front aero area = %.4f m^2\n", x_ms(1));
fprintf("Best rear aero area  = %.4f m^2\n", x_ms(2));
fprintf("Best total score     = %.4f points\n", score_ms);
fprintf("Exit flag            = %d\n", exitflag_ms);
fprintf("Local solutions found = %d\n", length(solutions_ms));