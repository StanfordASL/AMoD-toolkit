function electricity_cost_usd = ComputeElectricityCost(obj,varargin)
% ComputeElectricityCost Returns the cost of the electricity consumed by the electric AMoD system
%   electricity_cost_usd = ComputeElectricityCost(obj) uses the decision_vector in obj.decision_variables
%   electricity_cost_usd = ComputeElectricityCost(obj,decision_vector_val) uses decision_vector_val

% The natural way of doing this is as follows:
% charger_power_demand_w = obj.GetChargerPowerDemand();
% charger_electricity_demand_j = obj.spec.time_step_s*charger_power_demand_w;
% charger_power_price_usd_per_j = obj.spec.charger_power_price_usd_per_j;
% charger_electricity_cost_usd_x = charger_electricity_demand_j.*charger_power_price_usd_per_j;

% This has complicated numerics. We have that charger_power_demand_w = A_charger_power_normalized*obj.decision_variables.decision_vector 
% followed by a reshaping step, but A_charger_power_normalized has large values. Instead, we do as follows:
% charger_electricity_cost_usd = (factor_charger_power_demand_w_to_electricity_cost_usd*A_charger_power_normalized)*obj.decision_variables.decision_vector (with some reshaping).
% The matrix (factor_charger_power_demand_w_to_electricity_cost_usd*A_charger_power_normalized) has entries which are much closer to 1.
% This leads to better numerics

factor_charger_power_demand_w_to_electricity_cost_usd = obj.spec.time_step_s*obj.spec.charger_power_price_usd_per_j;
charger_electricity_cost_usd = obj.GetChargerPowerDemandNormalized(factor_charger_power_demand_w_to_electricity_cost_usd,varargin{:});

electricity_cost_usd = sum(charger_electricity_cost_usd(:));

end