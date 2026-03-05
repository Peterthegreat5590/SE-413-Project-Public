function [Lift, Drag] = aero_model(velocity, vehicle)
    rho = 1.225; %kg/m^3 density of air at sea level
    Lift = vehicle.CL*vehicle.AeroFrontalArea*rho*velocity^2/2;
    Drag = vehicle.CD*vehicle.AeroFrontalArea*rho*velocity^2/2;
end