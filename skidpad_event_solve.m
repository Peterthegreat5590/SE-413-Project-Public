function time = skidpad_event_solve(vehicle)

    velo_max = 5;
    df = 1;
    df_tol = 0.0001;
    while df>df_tol

        velo_max_new = lateral_velo_max(velo_max,vehicle);

        [~, drag] = aero_model(velo_max_new,vehicle);

        if drag > min(vehicle.Torque*vehicle.TireRadius,1000*vehicle.Power/velo_max_new)
            break
        else
            df = abs(velo_max_new-velo_max);
            velo_max = velo_max_new;
        end
    end

    disp(velo_max)

    time = (16.75*pi)/velo_max; 
end



function velo_max = lateral_velo_max(velocity, vehicle)
    lateral_accel_current = velocity^2/8.325; % v^2/r
    % Due to a prescribed equal roll stiffness front to rear, the
    % individual loads on the wheels can be calculated simply by the moment
    % arm ratios for each tire
    [lift, drag] = aero_model(velocity, vehicle);
    f_outer = -lift/2 + vehicle.Mass*9.81/2 + vehicle.Mass*lateral_accel_current*vehicle.CoGHeight/(vehicle.Trackwidth/2);
    f_inner = -lift/2 + vehicle.Mass*9.81/2 - vehicle.Mass*lateral_accel_current*vehicle.CoGHeight/(vehicle.Trackwidth/2);
    
    fo_tire_load = f_outer*vehicle.CoGFromRT/vehicle.Wheelbase;
    fi_tire_load = f_inner*vehicle.CoGFromRT/vehicle.Wheelbase;
    ro_tire_load = f_outer*(1-vehicle.CoGFromRT/vehicle.Wheelbase);
    ri_tire_load = f_inner*(1-vehicle.CoGFromRT/vehicle.Wheelbase);

    % Sample tire model at calculated corner loads
    [fo_lat, ~] = tire_model(fo_tire_load);
    [fi_lat, ~] = tire_model(fi_tire_load);
    [ro_lat, ro_lon] = tire_model(ro_tire_load);
    [ri_lat, ri_lon] = tire_model(ri_tire_load);

    % drag force reacted by rear tires reduces available lateral tire grip.
    % Tire grip envelope modelled as ellipse
    ro_lateral_max = sqrt((1-(drag/2)^2/ro_lon^2)*ro_lat^2);
    ri_lateral_max = sqrt((1-(drag/2)^2/ri_lon^2)*ri_lat^2);

    % Total lateral acceleration available from the maximum balanced moment
    % condition possible for the front and rear axle
    front_moment_max = (fo_lat+fi_lat)*(vehicle.Wheelbase-vehicle.CoGFromRT);
    rear_moment_max = (ro_lateral_max+ri_lateral_max)*vehicle.CoGFromRT;
    moment_max = min(front_moment_max,rear_moment_max);
    force_max = 2*moment_max/vehicle.Wheelbase;
    accel_max = force_max/vehicle.Mass;
    velo_max = sqrt(8.325*accel_max); %v = sqrt(r*a); a = v^2/r

end