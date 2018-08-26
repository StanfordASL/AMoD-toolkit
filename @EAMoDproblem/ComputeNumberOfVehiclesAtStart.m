function n_start_vehicles = ComputeNumberOfVehiclesAtStart(obj)
% ComputeNumberOfVehiclesAtStart Computes the number of vehicles present in the first time-step t = 1

n_start_vehicles = sum(obj.spec.initial_state_full_vehicles(:)) + sum(obj.spec.initial_state_empty_vehicles(:));
end