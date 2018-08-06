function n_start_vehicles = ComputeNumberOfVehiclesAtStart(obj)
n_start_vehicles = sum(obj.spec.FullVehicleInitialPos(:)) + sum(obj.spec.EmptyVehicleInitialPos(:));
end