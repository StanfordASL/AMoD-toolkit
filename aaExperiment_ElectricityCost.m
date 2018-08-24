function aaExperiment_ElectricityCost()

data = load('EAMoDspecDemo.mat');
spec = data.spec;
% Assuming a 50 kWh battery

spec.charge_unit_j = 50e3*3600/spec.C;
spec.time_step_s = 1800;

% Assuming a price of 43 usd/MWh
% This matrix needs to be transposed
spec.charger_power_price_usd_per_j = 43/(1e6*3600)*ones(spec.NumChargers,spec.Thor);

eamod_problem = EAMoDproblemBase(spec);

res = eamod_problem.Solve();

amod_cost = eamod_problem.ComputeAMoDcost();

% Compute electricity cost for EAMoD

% The natural way of doing this is as follows:
% charger_power_demand_w = eamod_problem.GetChargerPowerDemand();
% charger_electricity_demand_j = eamod_problem.spec.time_step_s*charger_power_demand_w;
% charger_power_price_usd_per_j = eamod_problem.spec.charger_power_price_usd_per_j;
% charger_electricity_cost_usd_x = charger_electricity_demand_j.*charger_power_price_usd_per_j;

% This has complicated numerics. We have that charger_power_demand_w = A_charger_power_normalized*obj.decision_variables.decision_vector 
% followed by a reshaping step, but A_charger_power_normalized has large values. Instead, we do as follows:
% charger_electricity_cost_usd = (factor_charger_power_demand_w_to_electricity_cost_usd*A_charger_power_normalized)*obj.decision_variables.decision_vector (with some reshaping).
% The matrix (factor_charger_power_demand_w_to_electricity_cost_usd*A_charger_power_normalized) has entries which are much closer to 1.
% This leads to better numerics

factor_charger_power_demand_w_to_electricity_cost_usd = eamod_problem.spec.time_step_s*eamod_problem.spec.charger_power_price_usd_per_j;
charger_electricity_cost_usd = eamod_problem.GetChargerPowerDemandNormalized(factor_charger_power_demand_w_to_electricity_cost_usd);

electricity_cost_usd = sum(charger_electricity_cost_usd(:));


end