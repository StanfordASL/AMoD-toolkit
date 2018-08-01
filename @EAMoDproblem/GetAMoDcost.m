% This cost excludes electricity cost
function [total_cost, pax_cost,reb_cost, relax_cost] = GetAMoDcost(obj)
state_vector = obj.decision_variables.state_vector;

[f_cost,f_cost_pax,f_cost_reb,f_cost_relax] = obj.CreateCostVector();

% Keep in mind that f_cost = f_cost_pax + f_cost_reb + f_cost_relax
total_cost = f_cost.'*state_vector;

pax_cost = f_cost_pax.'*state_vector;
reb_cost = f_cost_reb.'*state_vector;
relax_cost = f_cost_relax.'*state_vector;
end