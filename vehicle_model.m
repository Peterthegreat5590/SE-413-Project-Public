function vehicle = vehicle_model(vehicle)
    vehicle.Mass = vehicle.BareMass + vehicle.FrontAeroWeightNorm*vehicle.FrontAeroFrontalArea + vehicle.RearAeroWeightNorm*vehicle.RearAeroFrontalArea;
    if vehicle.FrontAeroFrontalArea+vehicle.RearAeroFrontalArea > 0
        vehicle.CoPFromRT = (vehicle.CLF*vehicle.FrontAeroFrontalArea*vehicle.lLF-vehicle.CLR*vehicle.RearAeroFrontalArea*vehicle.lLR)/(vehicle.CLF*vehicle.FrontAeroFrontalArea+vehicle.CLR*vehicle.RearAeroFrontalArea);
        vehicle.CoPHeight = (vehicle.CDF*vehicle.FrontAeroFrontalArea*vehicle.hDF+vehicle.CDR*vehicle.RearAeroFrontalArea*vehicle.hDR)/(vehicle.CDF*vehicle.FrontAeroFrontalArea+vehicle.CDR*vehicle.RearAeroFrontalArea);
    else
        vehicle.CoPFromRT = 0;
        vehicle.CoPHeight = 0;
    end
end