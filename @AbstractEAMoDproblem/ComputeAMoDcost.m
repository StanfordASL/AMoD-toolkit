function [amod_cost_usd, pax_cost_usd,reb_cost_usd, relax_cost_usd] = ComputeAMoDcost(obj,varargin)
% ComputeAMoDcost Returns the cost of operating the AMoD system excluding electricity
%   [amod_cost_usd, pax_cost_usd,reb_cost_usd, relax_cost_usd] = ComputeAMoDcost(obj) uses the state_vector in obj.optimization_variables
%   [amod_cost_usd, pax_cost_usd,reb_cost_usd, relax_cost_usd] = ComputeAMoDcost(obj,state_vector_val) uses state_vector_val
%   For the meaning of total_cost, pax_cost, reb_cost and relax_cost see EAMoDproblem.CreateCostVector
%
%   See also AbstractEAMoDproblem.CreateCostVector

switch numel(varargin)
    case 0
        state_vector = obj.optimization_variables.state_vector;
    case 1
        state_vector = varargin{1};
    otherwise
        error('Too many arguments.')
end

[~,f_cost_pax,f_cost_reb,f_cost_relax] = obj.CreateCostVector();

pax_cost_usd = f_cost_pax.'*state_vector;

reb_cost_usd = f_cost_reb.'*state_vector;

relax_cost_usd = f_cost_relax.'*state_vector;

amod_cost_usd = pax_cost_usd + reb_cost_usd + relax_cost_usd;
end