function velocity = inverse_aero_drag_model(drag, vehicle)
    rho = 1.225; %kg/m^3 density of air at sea level
    velocity = sqrt(2*drag/(vehicle.CDF*vehicle.FrontAeroFrontalArea*rho+vehicle.CDR*vehicle.RearAeroFrontalArea*rho));
end