function [amod_cost_usd, pax_cost_usd,reb_cost_usd, relax_cost_usd] = ComputeAMoDcost(obj,varargin)
% ComputeAMoDcost Overrides AbstractEAMoDproblem.ComputeAMoDcost to reflect the cost for pre-routed passenger-carrying vehicles 
%
%   See also AbstractEAMoDproblem.ComputeAMoDcost

[~, ~,reb_cost_usd, relax_cost_usd] = ComputeAMoDcost@AbstractEAMoDproblem(obj,varargin{:});

% When using the real time formulation, pax_cost_usd is constant
pax_cost_usd = obj.ComputePaxCostRealTimeFormulation();

amod_cost_usd = pax_cost_usd + reb_cost_usd + relax_cost_usd;
end