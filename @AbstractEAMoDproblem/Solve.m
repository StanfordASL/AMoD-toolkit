function [objective_value,solver_time_s,diagnostics] = Solve(obj)
% Solve Solves the optimization problem
%   [objective_value,solver_time_s,diagnostics] = Solve(obj)
%   objective_value is the value of the objective function at the optimum
%   solver_time_s is the solver time reported by YALMIP in seconds
%   diagnostics contains YALMIP diagnostic information

obj.optimization_variables = obj.DefineDecisionVariables();

constraint_array = obj.GetConstraintArray();
objective = obj.GetObjective();

diagnostics = optimize(constraint_array,objective,obj.yalmip_settings);

if diagnostics.problem ~= 0
    warning('Solver experienced issues: %s',diagnostics.info);
end

objective_value = value(objective);

solver_time_s = diagnostics.solvertime;
end