function [total_cost_val, pax_cost_val, reb_cost_val,relax_cost_val] = EvaluateAMoDcost(obj)

[total_cost, pax_cost,reb_cost, relax_cost] = obj.GetAMoDcost();

total_cost_val = value(total_cost);
pax_cost_val = value(pax_cost);
reb_cost_val = value(reb_cost);
relax_cost_val = value(relax_cost);
end