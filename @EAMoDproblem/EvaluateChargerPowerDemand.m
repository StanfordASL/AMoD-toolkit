function charger_power_demand_w_val = EvaluateChargerPowerDemand(obj,varargin)
switch numel(varargin)
    case 0
        decision_vector_val = obj.EvaluateDecisionVector();
    case 1
        decision_vector_val = varargin{1};
    otherwise
        error('Too many arguments.')
end

A_charger_power_w = obj.ComputeChargerPowerMatrixNew();


charger_power_demand_vec_w_val = A_charger_power_w*decision_vector_val;

charger_power_demand_w_val = reshape(charger_power_demand_vec_w_val,obj.spec.NumChargers,obj.spec.Thor);
end