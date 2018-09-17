classdef EAMoDspec
    % EAMoDspec Specifies an electric AMoD problem (EAMoDproblem)
    %   See also EAMoDproblem
    methods
        function obj = EAMoDspec()
            % EAMoDspec Constructs an empty object.  All fields without default values must be manually filled to fully specify the problem.
        end
        
        [time_range,label] = GetTimeRange(obj)
        obj = RemoveTripsEndingTooLate(obj)
        ValidateSpec(obj)
        
        % Get methods for dependent properties
        function n_time_step = get.n_time_step(obj)
            n_time_step = obj.n_time_step;
        end
        
        % It is sufficient to look at the initial conditions because
        % vehicles are conserved
        function n_vehicle = get.n_vehicle(obj)
            n_vehicle_full_init = sum(obj.initial_state_full_vehicles(:));
            n_vehicle_empty_init = sum(obj.initial_state_empty_vehicles(:));
            
            n_vehicle = n_vehicle_full_init + n_vehicle_empty_init;
        end
        
        function n_road_node = get.n_road_node(obj)
            n_road_node = numel(obj.road_adjacency_list);
        end
        
        function n_passenger_flow = get.n_passenger_flow(obj)
            n_passenger_flow = numel(obj.passenger_sink_list);
        end
        
        function n_passenger_sink = get.n_passenger_sink(obj)
            n_passenger_sink = numel(obj.passenger_sink_list);
        end
        
        function n_charger = get.n_charger(obj)
            n_charger = numel(obj.charger_list);
        end
        
        function obj = set.road_adjacency_list(obj,road_adjacency_list)         
            % Clean up road graph.
            for i = 1:numel(road_adjacency_list)
                road_adjacency_list{i} = sort(unique(road_adjacency_list{i}));
            end
            obj.road_adjacency_list = road_adjacency_list(:);
            
            obj.ValidateRoadGraph();
            
            obj = obj.UpdatePropertiesDependentOnRoadGraph();
        end
        
        function obj = set.passenger_source_list_cell(obj,passenger_source_list_cell)
            obj.passenger_source_list_cell = passenger_source_list_cell;
            
            obj = obj.UpdatePropertiesDependentOnSources();
        end        
    end
    
    methods (Static)
        obj = CreateFromScenario(scenario);
    end
        
    properties
        n_time_step(1,1) double {mustBeNonnegative,mustBeReal,mustBeInteger} % Number of time steps
        n_charge_step(1,1) double {mustBeNonnegative,mustBeReal,mustBeInteger} % Number of charge levels
        
        % Road network
        % Note: the empty lines after the group comments are intentional.
        % If they are not there the class documentation takes the comment
        % above the property instead of after it.
        
        road_adjacency_list(:,1) cell % road_adjacency_list{i} contains the neighbors of i in the road graph
                   
        road_capacity_matrix(:,:) double {mustBeNonnegative,mustBeReal} % road_capacity_matrix(i,j) is the capacity of the i-j link (in vehicles per unit time)
        road_travel_time_matrix(:,:) double {mustBeNonnegative,mustBeReal,mustBeInteger} % road_travel_time_matrix(i,j) is the travel time along the i-j link
        road_travel_distance_matrix_m(:,:) double {mustBeNonnegative,mustBeReal} % road_travel_distance_matrix_m(i,j) is the travel distance along the i-j link.
        road_charge_to_traverse_matrix(:,:) double {mustBeNonnegative,mustBeReal,mustBeInteger} % road_charge_to_traverse_matrix(i,j) is the amount of units of charge required to travel from i to j
        
        % Charger data
        
        charger_list(:,1) double {mustBeNonnegative,mustBeReal,mustBeInteger} % charger_list(l) is the location of the node in road_adjacency_list corresponding to the l-th charging station
        charger_speed(:,1) double {mustBeNonnegative,mustBeReal} % charger_speed(l) is the amount of charge gained by a vehicle crossing charging station l
        charger_time(:,1) double {mustBeNonnegative,mustBeReal} % charger_time(l) is the time required to charge charger_speed(l) units of charge at charging station l
        charger_capacity(:,1) double {mustBeNonnegative,mustBeReal} % charger_capacity(l) is the number of vehicles that can charge concurrently in charging station l
        
        charger_electricity_price_usd_per_j(:,:) double {mustBeReal} % charger_electricity_price_usd_per_j(l,t) is the price paid at time-step t for 1 Joule of energy consumed in charger l
        
        % Trip demand
        
        passenger_sink_list(:,1) double {mustBeNonnegative,mustBeReal,mustBeInteger} % passenger_sink_list(i) is the sink node of the i-th flow
        passenger_source_list_cell(:,1) cell % passenger_source_list_cell{i}(j) is the j-th origin leaving at time passenger_start_time_list_cell{i}(j) for passenger_sink_list(i)
        passenger_start_time_list_cell(:,1) cell % passenger_start_time_list_cell{i}(j) is the departure time for the flow from passenger_source_list_cell{i}(j) towards passenger_sink_list(i)
        passenger_flow_list_cell(:,1) cell % passenger_flow_list_cell{i}(j) is the amount of vehicle flow entering the flow of passenger_source_list_cell{i}(j) at time passenger_start_time_list_cell{i}(j) bound for passenger_sink_list(i).

        % Initial conditions
        
        initial_state_full_vehicles(:,:,:) double {mustBeNonnegative,mustBeReal} % initial_state_full_vehicles(m,i,c) is the number of vehicles with pax m at node i and charge c at time t=1
        initial_state_empty_vehicles(:,:) double {mustBeNonnegative,mustBeReal}% initial_state_empty_vehicles(i,c) is the number of empty vehicles at node i and charge c at time t=1
        
        % Final conditions
        
        final_min_charge(1,1) double {mustBeNonnegative,mustBeReal,mustBeInteger} % Minimum charge (in charge levels) of the rebalancing vehicles at the end of the simulation (value is exclusive. For example, setting it to 1 requires a minumum charge of 2.)
        
        % Economic parameters
        
        value_of_time_usd_per_time_step(1,1) double {mustBeNonnegative,mustBeReal} % Value in USD of one unit of time for a passenger
        vehicle_cost_usd_per_m(1,1) double {mustBeNonnegative,mustBeReal} % Cost in USD of running a vehicle for 1 m, excluding electricity cost
        battery_depreciation_usd_per_charge_step(1,1) double {mustBeNonnegative,mustBeReal} = 0 % Battery depreciation cost in USD due to charging or discharging by one unit of charge
                
        % Others
        
        time_step_s(1,1) double {mustBeNonnegative,mustBeReal} = 0 % Duration in seconds of a time step
        charge_step_j(1,1) double {mustBeNonnegative,mustBeReal} % Energy of a charge unit in joule, 1 kWh = 3.6e6 J
        v2g_efficiency(1,1) double {mustBeNonnegative,mustBeReal,mustBeLessThanOrEqual(v2g_efficiency,1)} = 1 % Efficiency of sending power back to the grid (vehicle2grid)
        
        disable_v2g(1,1) logical = false % Flag to disable v2g.
        
        start_date_time(1,1) datetime = datetime('01-Jan-2010 00:00:00'); % Starting time of the simulation
    end
    
    properties (GetAccess = public, SetAccess = private)
        % The following properties depend on others. We do not use dependent
        % properties so that we can cache them.
        
        % Dependent on road_adjacency_list
        road_adjacency_matrix(:,:) logical % road_adjacency_matrix(i,j) == 1 if there is a road going from node i to node j
        n_road_edge(1,1) double % Number of road edges in road_adjacency_list
        
        % Dependent on passenger_source_list_cell
        n_passenger_source(1,1) double % Total number of sources
    end
    
    properties (Dependent)
        n_road_node % Number of road nodes
        
        n_passenger_sink % Number of sinks
        n_passenger_flow % Number of bundled trip requests
        
        n_charger % Number of chargers
        
        n_vehicle % Number of vehicles
    end
    
    
    methods (Access = private)
        obj = UpdatePropertiesDependentOnRoadGraph(obj)
        
        obj = UpdatePropertiesDependentOnSources(obj)
        
        ValidateConsistencyWithAdjancencyMatrix(obj,n_passenger_flow,varargin)
        
        ValidateRoadGraph(obj)
        
        function res = IsValidRoadNode(obj,i)
            res = IsInteger(i) & 1 <= i & i <= obj.n_road_node;
        end
    end
    
    properties (Access = {?AbstractEAMoDproblem})
        % The following properties depend on others. We do not use dependent
        % properties so that we can cache them.
        
        % Dependent on road_adjacency_list
        road_reverse_adjacency_list(:,1) cell % Nodes in road_reverse_adjacency_list{i} are such that road_adjacency_list{road_reverse_adjacency_list{i}} contains i
        road_node_outdegree(:,1) double % road_node_outdegree(i) is the number of edges starting in road node i
        edge_number_matrix(:,:) double % edge_number_matrix(i,j) is the number of edge i-j, where edges are numbered as they appear in road_adjacency_list. It is nan if i-j is not an edge.
        
        % Dependent on passenger_source_list_cell
        n_sources_to_sink(:,1) double % n_sources_to_sink(i) is the number of passenger_source_list_cell towards passenger_sink_list(i)
        n_sources_to_sink_cumsum(:,1) double % Cumulative sum of n_sources_to_sink        
    end
    
end