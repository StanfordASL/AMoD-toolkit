classdef EAMoDspec
    % EAMoDspec Specifies an electric AMoD problem (EAMoDproblem)
    %   See also EAMoDproblem
    methods
        function obj = EAMoDspec()
            % EAMoDspec Constructs an empty object.  All fields without default values must be manually filled to fully specify the problem.
        end
        
        ValidateSpec(obj)
        
        [time_range,label] = GetTimeRange(obj)
        
        % Get methods for dependent properties
        function n_time_step = get.n_time_step(obj)
            n_time_step = obj.Thor;
        end
        
        % It is sufficient to look at the initial conditions because
        % vehicles are conserved
        function n_vehicle = get.n_vehicle(obj)
            n_vehicle_full_init = sum(obj.FullVehicleInitialPos(:));
            n_vehicle_empty_init = sum(obj.EmptyVehicleInitialPos(:));
            
            n_vehicle = n_vehicle_full_init + n_vehicle_empty_init;
        end
        
        function N = get.N(obj)
            N = numel(obj.RoadGraph);
        end
        
        function M = get.M(obj)
            M = numel(obj.Sinks);
        end
        
        function NumSinks = get.NumSinks(obj)
            NumSinks = numel(obj.Sinks);
        end
        
        function NumChargers = get.NumChargers(obj)
            NumChargers = numel(obj.ChargersList);
        end
        
        function obj = set.RoadGraph(obj,RoadGraph)         
            % Clean up road graph.
            for i = 1:numel(RoadGraph)
                RoadGraph{i} = sort(unique(RoadGraph{i}));
            end
            obj.RoadGraph = RoadGraph(:);
            
            obj.ValidateRoadGraph();
            
            obj = obj.UpdatePropertiesDependentOnRoadGraph();
        end
        
        function obj = set.Sources(obj,Sources)
            obj.Sources = Sources;
            
            obj = obj.UpdatePropertiesDependentOnSources();
        end        
    end
    
    methods (Static)
        obj = CreateFromScenario(scenario);
    end
        
    properties
        Thor(1,1) double {mustBeNonnegative,mustBeReal,mustBeInteger} % Number of time steps
        C(1,1) double {mustBeNonnegative,mustBeReal,mustBeInteger} % Number of charge levels
        
        % Road network
        % Note: the empty lines after the group comments are intentional.
        % If they are not there the class documentation takes the comment
        % above the property instead of after it.
        
        RoadGraph(:,1) cell % RoadGraph{i} contains the neighbors of i in the road graph
        
           
        RoadCap(:,:) double {mustBeNonnegative,mustBeReal} % RoadCap(i,j) is the capacity of the i-j link (in vehicles per unit time)
        TravelTimes(:,:) double {mustBeNonnegative,mustBeReal,mustBeInteger} % TravelTimes(i,j) is the travel time along the i-j link
        TravelDistance(:,:) double {mustBeNonnegative,mustBeReal} % TravelDistance(i,j) is the travel distance along the i-j link.
        ChargeToTraverse(:,:) double {mustBeNonnegative,mustBeReal,mustBeInteger} % ChargeToTraverse(i,j) is the amount of units of charge required to travel from i to j
        
        % Charger data
        
        ChargersList(:,1) double {mustBeNonnegative,mustBeReal,mustBeInteger} % ChargersList(l) is the location of the node in RoadGraph corresponding to the l-th charging station
        ChargerSpeed(:,1) double {mustBeNonnegative,mustBeReal} % ChargerSpeed(l) is the amount of charge gained by a vehicle crossing charging station l
        ChargerTime(:,1) double {mustBeNonnegative,mustBeReal} % ChargerTime(l) is the time required to charge ChargerSpeed(l) units of charge at charging station l
        ChargerCap(:,1) double {mustBeNonnegative,mustBeReal} % ChargerCap(l) is the number of vehicles that can charge concurrently in charging station l
        
        % TODO: rename to electricity price
        charger_power_price_usd_per_j(:,:) double {mustBeReal} % charger_power_price_usd_per_j(l,t) is the price paid at time-step t for 1 Joule of energy consumed in charger l
        
        % Trip demand
        
        Sinks(:,1) double {mustBeNonnegative,mustBeReal,mustBeInteger} % Sinks(i) is the sink node of the i-th flow
        Sources(:,1) cell % Sources{i}(j) is the j-th origin leaving at time StartTimes{i}(j) for Sinks(i)
        StartTimes(:,1) cell % StartTimes{i}(j) is the departure time for the flow from Sources{i}(j) towards Sinks(i)
        Flows(:,1) cell % Flows{i}(j) is the amount of vehicle flow entering the flow of Sources{i}(j) at time StartTimes{i}(j) bound for Sinks(i).

        % Initial conditions
        
        FullVehicleInitialPos(:,:,:) double {mustBeNonnegative,mustBeReal} % FullVehicleInitialPos(m,i,c) is the number of vehicles with pax m at node i and charge c at time t=1
        EmptyVehicleInitialPos(:,:) double {mustBeNonnegative,mustBeReal}% EmptyVehicleInitialPos(i,c) is the number of empty vehicles at node i and charge c at time t=1
        
        % Final conditions
        
        MinEndCharge(1,1) double {mustBeNonnegative,mustBeReal,mustBeInteger} % Minimum charge (in charge levels) of the rebalancing vehicles at the end of the simulation (value is exclusive. For example, setting it to 1 requires a minumum charge of 2.)
        
        % Economic parameters
        
        ValueOfTime(1,1) double {mustBeNonnegative,mustBeReal} % Value in USD of one unit of time for a passenger
        VehicleCostPerM(1,1) double {mustBeNonnegative,mustBeReal} % Cost in USD of running a vehicle for 1 m, excluding electricity cost
        BatteryDepreciationPerUnitCharge(1,1) double {mustBeNonnegative,mustBeReal} = 0 % Battery depreciation cost in USD due to charging or discharging by one unit of charge
                
        % Others
        
        time_step_s(1,:) double {mustBeNonnegative,mustBeReal,NumelMustBeLessThanOrEqual(time_step_s,1)} % Duration in seconds of a time step
        charge_unit_j(1,1) double {mustBeNonnegative,mustBeReal} % Energy of a charge unit in joule, 1 kWh = 3.6e6 J
        v2g_efficiency(1,1) double {mustBeNonnegative,mustBeReal,mustBeLessThanOrEqual(v2g_efficiency,1)} = 1 % Efficiency of sending power back to the grid (vehicle2grid)
        
        disable_v2g(1,1) logical = false % Flag to disable v2g.
        
        start_date_time(1,:) datetime {NumelMustBeLessThanOrEqual(start_date_time,1)} % Starting time of the simulation
    end
    
    properties (GetAccess = public, SetAccess = private)
        % The following properties depend on others. We do not use dependent
        % properties so that we can cache them.
        
        % Dependent on RoadGraph
        adjacency_matrix(:,:) logical % adjacency_matrix(i,j) == 1 if there is a road going from node i to node j
        E(1,1) double % Number of road edges in RoadGraph
        
        % Dependent on Sources
        TotNumSources(1,1) double % Total number of sources
    end
    
    properties (Dependent)
        N % Number of road nodes
        
        NumSinks % Number of sinks
        M % Number of bundled trip requests
        
        NumChargers % Number of chargers
        
        n_vehicle % Number of vehicles
        
        n_time_step % Alias for Thor
    end
    
    
    methods (Access = private)
        obj = UpdatePropertiesDependentOnRoadGraph(obj)
        
        obj = UpdatePropertiesDependentOnSources(obj)
        
        ValidateConsistencyWithAdjancencyMatrix(obj,M,varargin)
        
        ValidateRoadGraph(obj)
        
        function res = IsValidRoadNode(obj,i)
            res = IsInteger(i) & 1 <= i & i <= obj.N;
        end
    end
    
    properties (Access = {?EAMoDproblemBase})
        % The following properties depend on others. We do not use dependent
        % properties so that we can cache them.
        
        % Dependent on RoadGraph
        ReverseRoadGraph(:,1) cell % Nodes in ReverseRoadGraph{i} are such that RoadGraph{ReverseRoadGraph{i}} contains i
        NumRoadEdges(:,1) double % NumRoadEdges(i) is the number of edges starting in road node i
        cumRoadNeighbors(:,1) double % Cumulative sum of NumRoadEdges
        RoadNeighborCounter(:,:) double % Counter of road neighbors     
        
        % Dependent on Sources
        NumSourcesPerSink(:,1) double % NumSourcesPerSink(i) is the number of Sources towards Sinks(i)
        CumNumSourcesPerSink(:,1) double % Cumulative sum of NumSourcesPerSink        
    end
end