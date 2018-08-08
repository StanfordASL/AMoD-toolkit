function n_end_vehicles = ComputeNumberOfVehiclesAtEnd(obj,varargin)
% ComputeNumberOfVehiclesAtStart Computes the number of vehicles present in the last time-step t = obj.spec.Thor
%   n_end_vehicles = ComputeNumberOfVehiclesAtEnd(obj) uses the decision_vector in obj.decision_variables
%   n_end_vehicles = ComputeNumberOfVehiclesAtEnd(obj,decision_vector_val) uses decision_vector_val

final_vehicle_distribution = obj.GetFinalVehicleDistribution(varargin{:});

n_end_vehicles = sum(final_vehicle_distribution(:));
end