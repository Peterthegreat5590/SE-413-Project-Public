function time = accel_event_solve(vehicle)
    distance = 0;
    velocity = 0;
    acceleration = 0;
    time = 0;
    dt = 0.0001;
    while distance<75

        % RK4 approximation method
        k1 = acceleration_model(velocity, acceleration, vehicle);
        k2 = acceleration_model(velocity + dt/2*k1, k1, vehicle);
        k3 = acceleration_model(velocity + dt/2*k2, k2, vehicle);
        k4 = acceleration_model(velocity + dt*k3, k3, vehicle);
        
        acceleration = (k1+2*k2+2*k3+k4)/6;
        velocity = velocity + acceleration*dt;
        distance = distance + velocity*dt;
        time = time+dt;
        % if mod(time,1)<dt
        %     disp([time,velocity, acceleration]);
        % end
    end
end


function acceleration = acceleration_model(velocity, acceleration_old, vehicle)
        % Call Aero model to return lift and drag for tire loading and
        % drag for acceleration
        [lift, drag] = aero_model(velocity,vehicle);
        
        % Calculate total tire load from vehicle mass, downforce, and
        % weight transfer due to acceleration
        rear_tire_load = (-lift + vehicle.Mass*9.81)*(1-vehicle.CoGFromRT/vehicle.Wheelbase) + acceleration_old*vehicle.CoGHeight/vehicle.CoGFromRT;
        % Call Tire model to return maximum tire grip for acceleration
        [~, f_tire] = tire_model(rear_tire_load);
        f_torque = vehicle.Torque/vehicle.TireRadius;
        f_power = 1000*vehicle.Power/velocity;
        % Calculate maximum available force from tire, torque, and power
        % limits removing drag
        max_force = min([f_tire, f_torque, f_power]) - drag;
        acceleration = max_force/vehicle.Mass;
end



