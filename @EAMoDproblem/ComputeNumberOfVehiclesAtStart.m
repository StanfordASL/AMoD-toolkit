function n_start_vehicles = ComputeNumberOfVehiclesAtStart(obj)
n_start_vehicles = sum(obj.FullVehicleInitialPos(:)) + sum(obj.EmptyVehicleInitialPos(:));
end