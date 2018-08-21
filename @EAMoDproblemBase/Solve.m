function [objective_value,solver_time,diagnostics] = Solve(obj)
% Solve Solves the optimization problem
%   [objective_value,solver_time,diagnostics] = Solve(obj)
%   objective_value is the value of the objective function at the optimum
%   solver_time is the solver time reported by YALMIP
%   diagnostics contains YALMIP diagnostic information

obj.decision_variables = DefineDecisionVariables(obj);

constraint_array = obj.GetConstraintArray();
objective = obj.GetObjective();

diagnostics = optimize(constraint_array,objective,obj.yalmip_settings);

if diagnostics.problem ~= 0
    warning('Solver experienced issues: %s',diagnostics.info);
end

objective_value = value(objective);

solver_time = diagnostics.solvertime;
end