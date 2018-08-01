function E_charge_j = EvaluateChargerEnergyDemand(obj,varargin)
charger_power_demand_w_val = obj.EvaluateChargerPowerDemand(varargin{:});

charger_energy_demand_j_val = charger_power_demand_w_val*obj.time_step_s;

% Sum across chargers
E_charge_j = sum(charger_energy_demand_j_val,1);
end