function objective = GetObjective(obj)
% GetObjective Returns the optimization objective for the electric AMoD problem

amod_cost_usd = obj.ComputeAMoDcost();

if ~isempty(obj.spec.charger_power_price_usd_per_j)
    electricity_cost_usd = obj.ComputeElectricityCost();
else
    electricity_cost_usd = 0;
end

objective = amod_cost_usd + electricity_cost_usd;
end