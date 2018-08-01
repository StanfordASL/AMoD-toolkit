function HelperVerifyEnergyConservation(test_case,eamod_problem)
stored_energy_j = eamod_problem.ComputeStoredEnergy();

charger_power_demand_w = eamod_problem.GetChargerPowerDemand();

transportation_power_demand_w = eamod_problem.ComputeTransportationPowerDemand();



end