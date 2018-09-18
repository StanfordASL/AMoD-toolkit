function obj = CreateFromScenarioFR(scenario)
% CreateFromScenarioFR Instantiates EAMoDspec based on a legacy scenario (FR version)

RoadNetwork = scenario.RoadNetwork;
InitialConditions = scenario.InitialConditions;
Passengers = scenario.Passengers;
Flags = scenario.Flags;

obj = EAMoDspec();

obj.n_time_step = scenario.Thor;

obj.n_charge_step = RoadNetwork.C;

obj.road_adjacency_list = RoadNetwork.RoadGraph;

obj.passenger_source_list_cell = Passengers.Sources;
obj.passenger_sink_list = Passengers.Sinks;
obj.passenger_flow_list_cell = Passengers.Flows;
obj.passenger_start_time_list_cell = Passengers.StartTimes;

obj.road_capacity_matrix = RoadNetwork.RoadCap;
obj.road_travel_time_matrix = RoadNetwork.TravelTimes;
obj.road_travel_distance_matrix_m = RoadNetwork.TravelDistance;
obj.road_charge_to_traverse_matrix = RoadNetwork.ChargeToTraverse;

obj.charger_list = RoadNetwork.ChargersList;
obj.charger_time = RoadNetwork.ChargerTime;
obj.charger_speed = RoadNetwork.ChargerSpeed;
obj.charger_capacity = RoadNetwork.ChargerCap;

% Initial conditions
obj.initial_state_full_vehicles = InitialConditions.FullVehicleInitialPos;
obj.initial_state_empty_vehicles = InitialConditions.EmptyVehicleInitialPos;

% Final conditions
obj.final_min_charge = RoadNetwork.MinEndCharge;

% Economic settings
obj.value_of_time_usd_per_time_step = RoadNetwork.ValueOfTime;
obj.vehicle_cost_usd_per_m = RoadNetwork.VehicleCostPerKm/1000;

% Relaxation
if Flags.congrelaxflag
    warning('congrelaxflag is set but functionality is not available yet. Ignoring.')
end

% Extras
obj.v2g_efficiency = scenario.PowerNetwork.v2g_efficiency;

if isfield(scenario.RoadNetwork,'BatteryDepreciationPerUnitCharge')
    obj.battery_depreciation_usd_per_charge_step = scenario.RoadNetwork.BatteryDepreciationPerUnitCharge;
end
 
obj.time_step_s = scenario.time_step_s;
obj.charge_step_j = scenario.charge_unit_j;
    
end