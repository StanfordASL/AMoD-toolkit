scenario = LoadScenario('dfw_25cl_4h_30mstride_demo');

spec = EAMoDspec.CreateFromScenario(scenario);
spec = spec.RemoveTripsEndingTooLate();

spec.road_capacity_matrix = 12.5*3/4*spec.road_capacity_matrix;

initial_charge = 25;
initial_state_empty_vehicles = spec.initial_state_empty_vehicles;
spec.initial_state_empty_vehicles(:,initial_charge) = sum(initial_state_empty_vehicles,2);
% % Keep in mind that it is exclusive
spec.final_min_charge = 5;

save('EAMoDspecDemo_25n_8t_30c','spec')
