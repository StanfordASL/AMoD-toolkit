% This cost excludes electricity cost
function [total_cost, pax_cost,reb_cost, relax_cost] = GetAMoDcost(obj)
decision_vector = obj.decision_variables.decision_vector;

[f_cost,f_cost_pax,f_cost_reb,f_cost_relax] = obj.CreateCostVector();

% Keep in mind that f_cost = f_cost_pax + f_cost_reb + f_cost_relax
total_cost = f_cost.'*decision_vector;

pax_cost = f_cost_pax.'*decision_vector;
reb_cost = f_cost_reb.'*decision_vector;
relax_cost = f_cost_relax.'*decision_vector;
end