% charger_power_demand_w is a matrix with dimensions obj.spec.NumChargers,obj.spec.Thor
function charger_power_demand_w = GetChargerPowerDemand(obj,varargin)

% Setting the factor to 1 means no normalization
charger_power_demand_w = obj.GetChargerPowerDemandNormalized(1,varargin{:});
end

