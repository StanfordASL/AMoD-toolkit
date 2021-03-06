function [amod_cost_val_usd, pax_cost_val_usd, reb_cost_val_usd,relax_cost_val_usd] = EvaluateAMoDcost(obj,varargin)
% EvaluateAMoDcost Returns the value of the cost of operating the AMoD system excluding electricity
%   [amod_cost_val_usd, pax_cost_val_usd, reb_cost_val_usd,relax_cost_val_usd] = EvaluateAMoDcost(obj) uses the state_vector in obj.optimization_variables
%   [...] = EvaluateAMoDcost(obj,state_vector_val) uses state_vector_val
%   For the meaning of total_cost_val, pax_cost_val, reb_cost_val and relax_cost_val see EAMoDproblem.CreateCostVector
%
%   See also AbstractEAMoDproblem.CreateCostVector

[amod_cost_usd, pax_cost_usd,reb_cost_usd, relax_cost_usd] = obj.ComputeAMoDcost(varargin{:});

amod_cost_val_usd = value(amod_cost_usd);
pax_cost_val_usd = value(pax_cost_usd);
reb_cost_val_usd = value(reb_cost_usd);
relax_cost_val_usd = value(relax_cost_usd);
end