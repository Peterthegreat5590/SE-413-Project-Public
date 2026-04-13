function [Lift, Drag] = aero_model(velocity, vehicle)
    rho = 1.225; %kg/m^3 density of air at sea level
    Lift = vehicle.CLF*vehicle.FrontAeroFrontalArea*rho*velocity^2/2 + vehicle.CLR*vehicle.RearAeroFrontalArea*rho*velocity^2/2;
    Drag = vehicle.CDF*vehicle.FrontAeroFrontalArea*rho*velocity^2/2 + vehicle.CDR*vehicle.RearAeroFrontalArea*rho*velocity^2/2;
end