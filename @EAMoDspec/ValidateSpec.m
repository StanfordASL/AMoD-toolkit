function ValidateSpec(obj)
% ValidateSpec Asserts that the properties of the EAMoDspec object are consistent with each other

% Properties that depend on road_adjacency_list
obj.ValidateConsistencyWithAdjancencyMatrix(obj.road_capacity_matrix,'road_capacity_matrix');
obj.ValidateConsistencyWithAdjancencyMatrix(obj.road_travel_time_matrix,'road_travel_time_matrix');
obj.ValidateConsistencyWithAdjancencyMatrix(obj.road_travel_distance_matrix_m,'road_travel_distance_matrix_m');
obj.ValidateConsistencyWithAdjancencyMatrix(obj.road_charge_to_traverse_matrix,'road_charge_to_traverse_matrix');

% Properties that depend on charger_list
assert(numel(obj.charger_speed) == obj.n_charger,'charger_speed must have n_charger elements.');
assert(numel(obj.charger_time) == obj.n_charger,'charger_time must have n_charger elements');
assert(numel(obj.charger_time) == obj.n_charger,'charger_capacity must have n_charger elements');

assert(isempty(obj.charger_electricity_price_usd_per_j) || all(size(obj.charger_electricity_price_usd_per_j) == [obj.n_charger,obj.n_time_step]),'charger_electricity_price_usd_per_j must have size n_charger x n_time_step or be empty');


% Properties that depend on passenger_sink_list
assert(numel(obj.passenger_source_list_cell) == obj.n_passenger_sink,'passenger_source_list_cell must have n_passenger_sink elements');

% Properties that depend on passenger_source_list_cell
assert(all(cellfun(@numel,obj.passenger_start_time_list_cell) == obj.n_sources_to_sink),'passenger_start_time_list_cell must have the same number of cells and number of elements per cell as passenger_source_list_cell');
assert(all(cellfun(@(x) all(x >= 0) & all(x <= obj.n_time_step),obj.passenger_start_time_list_cell)),'All passenger_start_time_list_cell must be in range [0,n_time_step]');

assert(all(cellfun(@numel,obj.passenger_flow_list_cell) == obj.n_sources_to_sink),'passenger_flow_list_cell must have the same number of cells and number of elements per cell as passenger_source_list_cell');
assert(all(cellfun(@(x) all(x >= 0),obj.passenger_flow_list_cell)),'All passenger_flow_list_cell must be non-negative');

% Entries that must be valid road nodes
assert(all(obj.IsValidRoadNode(obj.charger_list)),'Entries in charger_list must be valid road nodes');
assert(all(obj.IsValidRoadNode(obj.passenger_sink_list)),'Entries in passenger_sink_list must be valid road nodes');

for i_sink = 1:obj.n_passenger_sink
    sources_towards_sink = obj.passenger_source_list_cell{i_sink};
    assert(all(obj.IsValidRoadNode(sources_towards_sink)),'Entries in passenger_source_list_cell{%i} must be valid road nodes',i_sink);
end

% Others
assert(obj.final_min_charge <= obj.n_charge_step - 1,'final_min_charge may not be more than n_charge_step - 1 (recall it is exclusive)');

assert(all(size(obj.initial_state_full_vehicles) == [obj.n_passenger_flow,obj.n_road_node,obj.n_charge_step]),'initial_state_full_vehicles must be of size n_passenger_flow x n_road_node x n_charge_step');
assert(all(size(obj.initial_state_empty_vehicles) == [obj.n_road_node,obj.n_charge_step]),'initial_state_full_vehicles must be of size n_road_node x n_charge_step');


end