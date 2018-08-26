function charger_power_demand_val_w = EvaluateChargerPowerDemand(obj,varargin)
% EvaluateChargerPowerDemand Returns the value of the power demand of the chargers used by the electric AMoD system
%   charger_power_demand_val_w = EvaluateChargerPowerDemand(obj) uses the state_vector in obj.optimization_variables
%   charger_power_demand_val_w = EvaluateChargerPowerDemand(obj,state_vector_val) uses state_vector_val
%   charger_power_demand_val_w(l,t) is the power demand in Watt of charger l at time step t
%
%   See also AbstractEAMoDproblem.ComputeChargerPowerDemand

charger_power_demand_w = obj.ComputeChargerPowerDemand(varargin{:});
charger_power_demand_val_w = value(charger_power_demand_w);
end