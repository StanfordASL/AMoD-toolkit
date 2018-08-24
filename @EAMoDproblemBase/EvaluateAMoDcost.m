function [total_cost_val, pax_cost_val, reb_cost_val,relax_cost_val] = EvaluateAMoDcost(obj,varargin)
% EvaluateAMoDcost Returns the value of the cost of operating the AMoD system excluding electricity
%   [total_cost_val, pax_cost_val, reb_cost_val,relax_cost_val] = EvaluateAMoDcost(obj) uses the decision_vector in obj.decision_variables
%   [total_cost_val, pax_cost_val, reb_cost_val,relax_cost_val] = EvaluateAMoDcost(obj,decision_vector_val) uses decision_vector_val
%   For the meaning of total_cost_val, pax_cost_val, reb_cost_val and relax_cost_val see EAMoDProblemBase.CreateCostVector
%   See also EAMoDProblemBase.CreateCostVector

[total_cost, pax_cost,reb_cost, relax_cost] = obj.ComputeAMoDcost(varargin{:});

total_cost_val = value(total_cost);
pax_cost_val = value(pax_cost);
reb_cost_val = value(reb_cost);
relax_cost_val = value(relax_cost);
end