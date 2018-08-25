function [amod_cost_usd, pax_cost_usd,reb_cost_usd, relax_cost_usd] = ComputeAMoDcost(obj,varargin)
% ComputeAMoDcost Returns the cost of operating the AMoD system excluding electricity
%   [amod_cost_usd, pax_cost_usd,reb_cost_usd, relax_cost_usd] = ComputeAMoDcost(obj) uses the decision_vector in obj.decision_variables
%   [amod_cost_usd, pax_cost_usd,reb_cost_usd, relax_cost_usd] = ComputeAMoDcost(obj,decision_vector_val) uses decision_vector_val
%   For the meaning of total_cost, pax_cost, reb_cost and relax_cost see EAMoDProblemBase.CreateCostVector
%   See also EAMoDProblemBase.CreateCostVector

switch numel(varargin)
    case 0
        decision_vector = obj.decision_variables.decision_vector;
    case 1
        decision_vector = varargin{1};
    otherwise
        error('Too many arguments.')
end

[~,f_cost_pax,f_cost_reb,f_cost_relax] = obj.CreateCostVector();

if obj.use_real_time_formulation
    % When using the real time formulation, pax_cost_usd is constant
    pax_cost_usd = obj.ComputePaxCostRealTimeFormulation();
else
    pax_cost_usd = f_cost_pax.'*decision_vector;
end

reb_cost_usd = f_cost_reb.'*decision_vector;

relax_cost_usd = f_cost_relax.'*decision_vector;

% Keep in mind that f_cost = f_cost_pax + f_cost_reb + f_cost_relax
amod_cost_usd = pax_cost_usd + reb_cost_usd + relax_cost_usd;
end