function electricity_cost_val_usd = EvaluateElectricityCost(obj,varargin)
% EvaluateElectricityCost Returns the value of the cost of the electricity consumed by the electric AMoD system
%   electricity_cost_val_usd = EvaluateElectricityCost(obj) uses the state_vector in obj.optimization_variables
%   electricity_cost_val_usd = EvaluateElectricityCost(obj,state_vector_val) uses state_vector_val

electricity_cost_usd = ComputeElectricityCost(obj,varargin{:});

electricity_cost_val_usd = value(electricity_cost_usd);
end


