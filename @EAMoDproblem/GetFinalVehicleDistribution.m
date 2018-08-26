function final_vehicle_distribution = GetFinalVehicleDistribution(obj,varargin)
% GetFinalVehicleDistribution Returns a matrix indicating the state of the vehicles at the end of the time period (t = obj.spec.n_time_step)
%   final_vehicle_distribution = GetFinalVehicleDistribution(obj) uses the decision_vector in obj.decision_variables
%   final_vehicle_distribution = GetFinalVehicleDistribution(obj,decision_vector_val) uses decision_vector_val
%   final_vehicle_distribution(c,i) represents the number of vehicles in road node i with charge level c. 

switch numel(varargin)
    case 0
        decision_vector_val = obj.EvaluateDecisionVector();
    case 1
        decision_vector_val = varargin{1};
    otherwise
        error('Too many arguments.')
end

final_vehicle_distribution = zeros(obj.spec.n_charge_step,obj.spec.n_road_node);

for c = 1:obj.spec.n_charge_step
    for i = 1:obj.spec.n_road_node
        final_vehicle_distribution(c,i) = decision_vector_val(obj.FindEndRebLocationci(c,i));
    end
end

end