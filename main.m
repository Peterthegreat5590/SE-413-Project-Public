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
total_time = datetime - start_time;
total_time.Format = 'mm:ss.SSS';
disp(total_time)

disp(-f)
disp(x)

front = 0:0.1:1;
rear = 0:0.1:1;

[Fr, Re] = meshgrid(front, rear);

Ob = zeros(size(Fr));
for i = 1:length(rear)
    for j = 1:length(front)
        [score, ineq] = comp_solver(Fr(i,j),Re(i,j),vehicle);
        Ob(i,j) = score;
    end
end

hold on
contourf(Fr, Re, Ob, 21)
plot(x(1), x(2), "k.",MarkerSize=20)
c = colorbar();
c.Label.String = "Total Score";
xlabel("Front Aerodynamic Frontal Area (m^2)")
ylabel("Rear Aerodynamic Frontal Area (m^2)")
title("Predicted Score of a FSAE car vs Aerodynamic Surface Area")