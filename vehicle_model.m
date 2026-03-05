function vehicle = vehicle_model(vehicle)
    vehicle.Mass = vehicle.BareMass + vehicle.AeroWeightNorm*vehicle.AeroFrontalArea;
end