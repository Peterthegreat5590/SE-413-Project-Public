function [time, alpha, velo_max] = skidpad_event_solve(vehicle)

    velo_max = 5;
    df = 1;
    df_tol = 0.000001;

    while df>df_tol

        [velo_max_new, alpha] = lateral_velo_max(velo_max,vehicle);

        max_powertrain = min(vehicle.Torque*vehicle.TireRadius,1000*vehicle.Power/velo_max_new);
        velo_max_drag = inverse_aero_drag_model(max_powertrain, vehicle);
        
        if velo_max_new > velo_max_drag
            % disp("Drag limit reached")
            [velo_max_new, alpha] = lateral_velo_max(velo_max_drag, vehicle);

            max_powertrain = min(vehicle.Torque*vehicle.TireRadius,1000*vehicle.Power/velo_max_new);
            velo_max_drag = inverse_aero_drag_model(max_powertrain, vehicle);
            
            if velo_max_new > velo_max_drag
                velo_max = velo_max_drag;
                break
            end
        else
            df = abs(velo_max_new-velo_max);
            velo_max = velo_max_new;
        end
    end

    % disp(velo_max)

    time = (18.25*pi)/velo_max; 
end



function [velo_max, alpha] = lateral_velo_max(velocity, vehicle)
    lateral_accel_current = velocity^2/8.325; % v^2/r
    % Due to a prescribed equal roll stiffness front to rear, the
    % individual loads on the wheels can be calculated simply by the moment
    % arm ratios for each tire
    [lift, drag] = aero_model(velocity, vehicle);
    
    f_tire_load = (vehicle.Mass*9.81*vehicle.CoGFromRT - lift*vehicle.CoPFromRT - drag*vehicle.CoPHeight/2)/vehicle.Wheelbase;
    r_tire_load = (vehicle.Mass*9.81*(vehicle.Wheelbase-vehicle.CoGFromRT) - lift*(vehicle.Wheelbase-vehicle.CoPFromRT) + drag*vehicle.CoPHeight/2)/vehicle.Wheelbase;
    ro_tire_load = r_tire_load/2 + vehicle.Mass*lateral_accel_current*vehicle.CoGHeight/(vehicle.Trackwidth*2);
    ri_tire_load = r_tire_load/2 - vehicle.Mass*lateral_accel_current*vehicle.CoGHeight/(vehicle.Trackwidth*2);

    % Sample tire model at calculated corner loads
    [~, ro_lon] = tire_model(ro_tire_load);
    [~, ri_lon] = tire_model(ri_tire_load);


    function velo = velo_func(alpha)
        alpha_r = alpha(2);
        alpha_f = alpha(1);
        % drag force reacted by rear tires reduces available lateral tire grip.
        % Tire grip envelope modelled as ellipse
        ro_lateral = sqrt((1-(drag/2)^2/ro_lon^2))*slip_angle_tire_model(ro_tire_load, alpha_r);
        ri_lateral = sqrt((1-(drag/2)^2/ri_lon^2))*slip_angle_tire_model(ri_tire_load, alpha_r);
        % Total lateral acceleration available from the maximum balanced moment
        % condition possible for the front and rear axle
        f_lateral = slip_angle_tire_model(f_tire_load, alpha_f);
        front_moment = f_lateral*(vehicle.Wheelbase-vehicle.CoGFromRT);
        rear_moment = (ro_lateral+ri_lateral)*vehicle.CoGFromRT;
        moment_max = min(front_moment,rear_moment);
        force_max = 2*moment_max/vehicle.Wheelbase;
        accel_max = force_max/vehicle.Mass;
        velo = sqrt(9.125*accel_max); %v = sqrt(r*a); a = v^2/r
    end
    fopt = @(x) -velo_func(x);

    function [ineq, eq] = cons_func(alpha)
        alpha_r = alpha(2);
        alpha_f = alpha(1);
        ineq = [];
        ro_lateral = sqrt((1-(drag/2)^2/ro_lon^2))*slip_angle_tire_model(ro_tire_load, alpha_r);
        ri_lateral = sqrt((1-(drag/2)^2/ri_lon^2))*slip_angle_tire_model(ri_tire_load, alpha_r);
        f_lateral = slip_angle_tire_model(f_tire_load, alpha_f);
        front_moment = f_lateral*(vehicle.Wheelbase-vehicle.CoGFromRT);
        rear_moment = (ro_lateral+ri_lateral)*vehicle.CoGFromRT;
        eq = front_moment-rear_moment;
    end
    opts = optimoptions("fmincon","Display","none");
    alpha = fmincon(fopt,[0.15,0.15],[],[],[],[],[0,0],[pi/2,pi/2],@cons_func,opts);


    velo_max = velo_func(alpha);
end