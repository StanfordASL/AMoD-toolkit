function objective = GetObjective(obj)
% GetObjective Returns the optimization objective for the electric AMoD problem

amod_cost_usd = obj.ComputeAMoDcost();
electricity_cost_usd = obj.ComputeElectricityCost();

objective = amod_cost_usd + electricity_cost_usd;
end