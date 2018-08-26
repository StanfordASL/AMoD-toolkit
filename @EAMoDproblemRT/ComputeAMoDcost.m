function [amod_cost_usd, pax_cost_usd,reb_cost_usd, relax_cost_usd] = ComputeAMoDcost(obj,varargin)
% ComputeAMoDcost Overrides AbstractEAMoDproblem.ComputeAMoDcost to use the cost for pre-routed passenger-carrying vehicles 
%   [amod_cost_usd, pax_cost_usd,reb_cost_usd, relax_cost_usd] = ComputeAMoDcost(obj) uses the state_vector in obj.optimization_variables
%   [amod_cost_usd, pax_cost_usd,reb_cost_usd, relax_cost_usd] = ComputeAMoDcost(obj,decision_vector_val) uses decision_vector_val
%   For the meaning of total_cost, pax_cost, reb_cost and relax_cost see AbstractEAMoDproblem.CreateCostVector
%
%   See also AbstractEAMoDproblem.ComputeAMoDcost, AbstractEAMoDproblem.CreateCostVector

[~, ~,reb_cost_usd, relax_cost_usd] = ComputeAMoDcost@AbstractEAMoDproblem(obj,varargin{:});

% When using the real time formulation, pax_cost_usd is constant
pax_cost_usd = obj.ComputePaxCostRealTimeFormulation();

amod_cost_usd = pax_cost_usd + reb_cost_usd + relax_cost_usd;
end