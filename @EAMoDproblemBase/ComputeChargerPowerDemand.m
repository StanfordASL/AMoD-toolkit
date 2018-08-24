function charger_power_demand_w = ComputeChargerPowerDemand(obj,varargin)
% ComputeChargerPowerDemand Computes the power demand of the chargers used by the electric AMoD system
%   charger_power_demand_w = ComputeChargerPowerDemand(obj) uses the decision_vector in obj.decision_variables
%   charger_power_demand_w = ComputeChargerPowerDemand(obj,decision_vector_val) uses decision_vector_val
%   charger_power_demand_w(l,t) is the power demand in Watt of charger l at time step t
%   See also EAMoDProblemBase.GetChargerPowerDemandNormalized

% Setting the factor to 1 means no normalization
charger_power_demand_w = obj.GetChargerPowerDemandNormalized(1,varargin{:});
end

