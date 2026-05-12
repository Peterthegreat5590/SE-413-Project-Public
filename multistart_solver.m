function [x_ms, f_ms, exitflag_ms, output_ms, solutions_ms] = multistart_solver(vehicle, num_start_points, use_parallel)
    xLast = []; % Last place computeall was called
    myscore = []; % Use for objective at xLast
    myc = []; % Use for nonlinear inequality constraint

    if nargin < 2
        num_start_points = 8;
    end

    if nargin < 3
        use_parallel = false;   % Set true only if Parallel Computing Toolbox is available
    end

    fun = @objective_function;
    cfun = @constraint_function;

    max_front_area = 0.25*3 + 0.25*(3 - 0.407);
    max_rear_area  = (1.2 - 0.407)*(3 - 0.407);

    lb = [0, 0];
    ub = [max_front_area, max_rear_area];

    x0 = [0.5, 0.5];

    opts_fmincon = optimoptions("fmincon", ...
        "Display", "none", ...
        "Algorithm", "sqp", ...
        "FunctionTolerance", 1e-5, ...
        "StepTolerance", 1e-5, ...
        "OptimalityTolerance", 1e-5, ...
        "ConstraintTolerance", 1e-5);

    problem = createOptimProblem("fmincon", ...
        "objective", fun, ...
        "x0", x0, ...
        "lb", lb, ...
        "ub", ub, ...
        "nonlcon", cfun, ...
        "options", opts_fmincon);

    % Create MultiStart object
    ms = MultiStart("Display", "off", "UseParallel", use_parallel);

    % Run MultiStart
    [x_ms, f_ms, exitflag_ms, output_ms, solutions_ms] = run(ms, problem, num_start_points);

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

