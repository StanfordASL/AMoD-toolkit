function stored_energy_j = ComputeStoredEnergyAtEnd(obj)

final_vehicle_distribution = obj.GetFinalVehicleDistribution();

stored_charge = 0;

for c = 1:obj.spec.C
    final_vehicle_distribution_c = final_vehicle_distribution(c,:);
    stored_charge = stored_charge + c*sum(final_vehicle_distribution_c(:));
end

stored_energy_j = stored_charge*obj.spec.charge_unit_j;
end