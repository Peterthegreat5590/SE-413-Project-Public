function [x, f, exitflag, output] = edo_problem_solver(vehicle)

xLast = []; % Last place computeall was called
myscore = []; % Use for objective at xLast
myc = []; % Use for nonlinear inequality constraint

fun = @objective_function;
cfun = @constraint_function;


max_front_area = 0.25*3 + 0.25*(3-0.407); % See FSAE Rules 2026 V1 T.7.7 for more info
max_rear_area = (1.2-0.407)*(3-0.407);

% Placeholder for Optimization solver
opts = optimoptions("fmincon","FunctionTolerance",1e-4,"StepTolerance",1e-4,"OptimalityTolerance",1e-4);
[x, f, exitflag, output] = fmincon(fun,[0.5,0.5],[],[],[],[],[0,0],[max_front_area, max_rear_area],cfun,opts);


function y = objective_function(x)
    if ~isequal(x,xLast) % Check if computation is necessary
        front_aero_frontal_area = x(1);
        rear_aero_frontal_area = x(2);
        [myscore, myc] = comp_solver(front_aero_frontal_area, rear_aero_frontal_area, vehicle);
        xLast = x;
    end
    % Now compute objective function
    y = -myscore;
end

function [ineq, eq] = constraint_function(x)
    if ~isequal(x,xLast) % Check if computation is necessary
        front_aero_frontal_area = x(1);
        rear_aero_frontal_area = x(2);
        [myscore, myc] = comp_solver(front_aero_frontal_area, rear_aero_frontal_area, vehicle);
        xLast = x;
    end
    ineq = myc;
    eq = [];

end
end