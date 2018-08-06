function [objective_value,solver_time,diagnostics]  = Solve(obj)

if ~obj.is_initialized
    obj.Initialize();
end

obj.constraint_array = obj.DefineConstraints();

objective = obj.GetAMoDcost();

sdp_options = sdpsettings('solver',obj.spec.solver);
diagnostics = optimize(obj.constraint_array,objective,sdp_options);

if diagnostics.problem ~= 0
    warning('Solver experienced issues: %s',diagnostics.info);
end

objective_value = value(objective);

solver_time = diagnostics.solvertime;
end