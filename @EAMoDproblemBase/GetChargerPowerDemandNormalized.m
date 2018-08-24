% A_charger_power_w can have large entries. The point of this function is
% to reduce this effect before multiplying it with the state vector. This
% improves numerical conditioning.
% charger_power_demand is a matrix with dimensions obj.spec.NumChargers,obj.spec.Thor
function charger_power_demand = GetChargerPowerDemandNormalized(obj,factor,varargin)
if ~isscalar(factor)
   assert(all(size(factor) == [obj.spec.NumChargers,obj.spec.Thor]),'factor must be a scalar or a matrix of size spec.NumChargers x spec.Thor.') 
end

switch numel(varargin)
    case 0
        decision_vector = obj.decision_variables.decision_vector;
    case 1
        decision_vector = varargin{1};
    otherwise
        error('Too many arguments.')
end

A_charger_power_w = obj.ComputeChargerPowerMatrixNew();

A_charger_power_normalized = spdiag(factor(:))*A_charger_power_w;

charger_power_demand_vec = A_charger_power_normalized*decision_vector;

charger_power_demand = reshape(charger_power_demand_vec,obj.spec.NumChargers,obj.spec.Thor);
end