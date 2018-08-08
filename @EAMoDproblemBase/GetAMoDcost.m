function [total_cost, pax_cost,reb_cost, relax_cost] = GetAMoDcost(obj,varargin)
% GetAMoDcost Returns the cost of operating the AMoD system excluding electricity
%   [total_cost, pax_cost,reb_cost, relax_cost] = GetAMoDcost(obj) uses the decision_vector in obj.decision_variables
%   [total_cost, pax_cost,reb_cost, relax_cost] = GetAMoDcost(obj,decision_vector_val) uses decision_vector_val
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

[f_cost,f_cost_pax,f_cost_reb,f_cost_relax] = obj.CreateCostVector();

% Keep in mind that f_cost = f_cost_pax + f_cost_reb + f_cost_relax
total_cost = f_cost.'*decision_vector;

pax_cost = f_cost_pax.'*decision_vector;
reb_cost = f_cost_reb.'*decision_vector;
relax_cost = f_cost_relax.'*decision_vector;
end