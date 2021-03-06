function charger_power_demand_w = ComputeChargerPowerDemand(obj,varargin)
% ComputeChargerPowerDemand Computes the power demand of the chargers used by the electric AMoD system
%   charger_power_demand_w = ComputeChargerPowerDemand(obj) uses the state_vector in obj.optimization_variables
%   charger_power_demand_w = ComputeChargerPowerDemand(obj,state_vector_val) uses state_vector_val
%   charger_power_demand_w(l,t) is the power demand in Watt of charger l at time step t
%
%   See also AbstractEAMoDproblem.ComputeChargerPowerDemandNormalized

% Setting the factor to 1 means no normalization
charger_power_demand_w = obj.ComputeChargerPowerDemandNormalized(1,varargin{:});
end

