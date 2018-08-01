function stored_energy_j = ComputeStoredEnergyAtStart(obj)

stored_charge_full_vehicles = 0;
stored_charge_empty_vehicles = 0;

for c = 1:obj.C
    full_vehicle_initial_pos_c = obj.FullVehicleInitialPos(:,:,c);
    stored_charge_full_vehicles = stored_charge_full_vehicles + c*sum(full_vehicle_initial_pos_c(:));
    
    empty_vehicle_initial_pos_c = obj.EmptyVehicleInitialPos(:,c);
    stored_charge_empty_vehicles = stored_charge_empty_vehicles + c*sum(empty_vehicle_initial_pos_c(:));
end

stored_charge = stored_charge_full_vehicles + stored_charge_empty_vehicles;

stored_energy_j = stored_charge*obj.charge_unit_j;
end