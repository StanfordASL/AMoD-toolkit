% A_charger_power_w can have large entries. The point of this function is
% to reduce this effect before multiplying it with the state vector. This
% improves numerical conditioning.
% charger_power_demand is a matrix with dimensions obj.spec.NumChargers,obj.spec.Thor
function charger_power_demand = GetChargerPowerDemandNormalized(obj,factor)
A_charger_power_w = obj.ComputeChargerPowerMatrixNew();

A_charger_power_normalized = factor*A_charger_power_w;

charger_power_demand_vec = A_charger_power_normalized*obj.decision_variables.decision_vector;

charger_power_demand = reshape(charger_power_demand_vec,obj.spec.NumChargers,obj.spec.Thor);
end