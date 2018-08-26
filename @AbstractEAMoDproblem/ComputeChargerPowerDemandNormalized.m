function charger_power_demand = ComputeChargerPowerDemandNormalized(obj,factor,varargin)
% ComputeChargerPowerDemandNormalized Computes charger power demand normalized with a matrix factor
%   charger_power_demand = ComputeChargerPowerDemandNormalized(obj,factor) uses the state_vector in obj.optimization_variables
%   charger_power_demand = ComputeChargerPowerDemandNormalized(obj,factor,decision_vector_val) uses decision_vector_val
%   factor can be a scalar or a matrix of size spec.n_charger x obj.spec.n_time_step
%   charger_power_demand(l,t) is the normalized power demand in charger l at time step t
%
%   The reason for normalization is as follows. Without normalization
%   charger_power_demand_vec_w = A_charger_power_w*state_vector 
%   where state_vector is an sdpvar and A_charger_power_w typically has 
%   very large entries (flow of 1 car corresponds to several kW load). This
%   often leads to numerical problems. Often, we do not care about Watts, 
%   but about per-unit power, energy, etc. For example, see EAMoDproblem.ComputeElectricityCost
%   In these cases we would do
%   charger_power_demand_w = ComputeChargerPowerDemand();
%   something_we_care_about = factor*charger_power_demand_w;
%   Instead, we can do:
%   something_we_care_about = ComputeChargerPowerDemandNormalized(factor)
%   and get better numerical behavior.
%
%   See also EAMoDproblem.ComputeElectricityCost

if ~isscalar(factor)
   assert(all(size(factor) == [obj.spec.n_charger,obj.spec.n_time_step]),'factor must be a scalar or a matrix of size spec.n_charger x spec.n_time_step.') 
end

switch numel(varargin)
    case 0
        state_vector = obj.optimization_variables.state_vector;
    case 1
        state_vector = varargin{1};
    otherwise
        error('Too many arguments.')
end

A_charger_power_w = obj.ComputeChargerPowerMatrix();

A_charger_power_normalized = spdiag(factor(:))*A_charger_power_w;

charger_power_demand_vec = A_charger_power_normalized*state_vector;

charger_power_demand = reshape(charger_power_demand_vec,obj.spec.n_charger,obj.spec.n_time_step);
end