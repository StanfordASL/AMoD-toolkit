% charger_power_demand_w is a matrix with dimensions obj.NumChargers,obj.Thor
function charger_power_demand_w = GetChargerPowerDemand(obj)
% Setting the factor to 1 means no normalization
charger_power_demand_w = obj.GetChargerPowerDemandNormalized(1);
end

