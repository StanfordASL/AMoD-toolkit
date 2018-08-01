function n_end_vehicles = ComputeNumberOfVehiclesAtEnd(obj,varargin)

final_vehicle_distribution = obj.GetFinalVehicleDistribution(varargin{:});

n_end_vehicles = sum(final_vehicle_distribution(:));

end