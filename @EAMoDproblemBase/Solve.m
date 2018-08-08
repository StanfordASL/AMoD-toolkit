function [objective_value,solver_time,diagnostics] = Solve(obj)

constraint_array = obj.GetConstraintArray();
objective = obj.GetObjective();

diagnostics = optimize(constraint_array,objective,obj.yalmip_settings);

if diagnostics.problem ~= 0
    warning('Solver experienced issues: %s',diagnostics.info);
end

objective_value = value(objective);

solver_time = diagnostics.solvertime;
end