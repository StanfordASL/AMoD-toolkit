function n_end_vehicles = ComputeNumberOfVehiclesAtEnd(obj,varargin)
% ComputeNumberOfVehiclesAtStart Computes the number of vehicles present in the last time-step t = obj.spec.n_time_step
%   n_end_vehicles = ComputeNumberOfVehiclesAtEnd(obj) uses the state_vector in obj.optimization_variables
%   n_end_vehicles = ComputeNumberOfVehiclesAtEnd(obj,state_vector_val) uses state_vector_val

final_vehicle_distribution = obj.GetFinalVehicleDistribution(varargin{:});

n_end_vehicles = sum(final_vehicle_distribution(:));
end