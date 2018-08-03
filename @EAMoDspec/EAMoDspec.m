classdef EAMoDspec
    methods
        function obj = EAMoDspec()
        end
        
        ValidateSpec(obj);
        
        function n_time_step = get.n_time_step(obj)
            n_time_step = obj.Thor;
        end
        
        % It is sufficient to look at the initial conditions because
        % vehicles are conserved
        function n_vehicle = get.n_vehicle(obj)
            n_vehicle_full_init = sum(obj.InitialConditions.FullVehicleInitialPos(:));
            n_vehicle_empty_init = sum(obj.InitialConditions.EmptyVehicleInitialPos(:));
            
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
            EAMoDspec.ValidateRoadGraph(RoadGraph);
            
            % Clean up road graph.
            for i = 1:numel(RoadGraph)
                RoadGraph{i} = sort(unique(RoadGraph{i}));
            end
            obj.RoadGraph = RoadGraph(:);
            
            obj = obj.UpdatePropertiesDependentOnRoadGraph();
        end
        
        function obj = set.Sources(obj,Sources)
            obj.Sources = Sources;
            
            obj = obj.UpdatePropertiesDependentOnSources();
        end
        
    end
    
    methods (Static)
        ValidateRoadGraph(RoadGraph)
    end
    
    properties (GetAccess = public, SetAccess = private)
        % The following properties depend on others. We do not use dependent
        % properties to that we can cache them.
        
        % Dependent on RoadGraph
        ReverseRoadGraph(:,1) cell % Nodes in ReverseRoadGraph{i} are such that RoadGraph{ReverseRoadGraph{i}} contains i
        adjacency_matrix(:,:) logical % adjacency_matrix(i,j) == 1 if there is a road going from node i to node j
        E(1,1) double % Number of road edges in RoadGraph
        NumRoadEdges(:,1) double % NumRoadEdges(i) is the number of edges starting in road node i
        cumRoadNeighbors(:,1) double
        RoadNeighborCounter(:,:) double
        
        % Dependent on Sources
        NumSourcesPerSink(:,1) double % NumSourcesPerSink(i) is the number of Sources towards Sinks(i)
        CumNumSourcesPerSink(:,1) double % Cumulative sum of NumSourcesPerSink
        TotNumSources(1,1) double % Total number of sources
    end
    
    properties (Dependent)
        N
        
        NumSinks
        M
        
        NumChargers
        
        n_vehicle
        
        % Alias
        n_time_step
    end
    
    properties
        % TODO: add comments to all properties
        Thor
        C(1,1) double {mustBeNonnegative,mustBeReal,mustBeInteger} % number of charge levels
        
        % Road network
        RoadGraph(:,1) cell% RoadGraph{i} contains the neighbors of i in the road graph
        
        RoadCap(:,:) double {mustBeNonnegative,mustBeReal} % RoadCap(i,j) is the capacity of the i-j link (in vehicles per unit time)
        TravelTimes(:,:) double {mustBeNonnegative,mustBeReal,mustBeInteger} % TravelTimes(i,j) is the travel time along the i-j link
        TravelDistance(:,:) double {mustBeNonnegative,mustBeReal} % TravelDistance(i,j) is the travel distance along the i-j link.
        ChargeToTraverse(:,:) double {mustBeNonnegative,mustBeReal,mustBeInteger} % ChargeReqs(i,j) is the amount of units of charge required to travel from i to j
        
        % Charger data
        ChargersList(:,1) double {mustBeNonnegative,mustBeReal,mustBeInteger} % ChargersList(i) is the location of the node in RoadGraph corresponding to the i-th charging station
        ChargerSpeed(:,1) double {mustBeNonnegative,mustBeReal} % ChargerSpeed(i) is the amount of charge gained by a vehicle crossing charging station i
        ChargerTime(:,1) double {mustBeNonnegative,mustBeReal} % ChargerTime(i) is the time required to charge ChargerSpeed(i) units of charge at charging station i
        ChargerCaps(:,1) double {mustBeNonnegative,mustBeReal} % ChargerCaps(i) is the number of vehicles that can charge concurrently in charging station i
        
        % Trip demand
        Sinks(:,1) double {mustBeNonnegative,mustBeReal,mustBeInteger} % Sinks(i) is the sink node of the i-th flow
        Sources(:,1) cell % Sources{i}(j) is the j-th origin leaving at time StartTimes{i}(j) for Sinks(i)
        StartTimes(:,1) cell % StartTimes{i}(j) is the departure time from source Sources{i}(j) for the flow from Sources{i}(j) towards Sinks(i)
        Flows(:,1) cell % Flows{i}(j) is the amount of vehicle flow entering the source of flow Sources{i}(j) at time StartTimes{i}(j) bound for Sinks(i).
        % TODO: verify consistent sizes, valid nodes and valid start times
        % and positive flows
        
        % Initial conditions
        FullVehicleInitialPos(:,:,:) double {mustBeNonnegative,mustBeReal} % FullVehicleInitialPos(m,i,c) is the number of vehicles with pax m at node i and charge c at time t=1
        EmptyVehicleInitialPos(:,:) double {mustBeNonnegative,mustBeReal}% EmptyVehicleInitialPos(i,c) is the number of empty vehicles at node i and charge c at time t=1
        
        % Final conditions
        MinEndCharge(1,1) double {mustBeNonnegative,mustBeReal,mustBeInteger} % Minimum charge (in charge levels) of the rebalancing vehicles at the end of the simulation (value is exclusive. For example, setting it to 1 requires a minumum charge of 2.)
        
        % Economic parameters
        ValueOfTime(1,1) double {mustBeNonnegative,mustBeReal} % Value (in USD) of one unit of time for a passenger
        VehicleCostPerKm(1,1) double {mustBeNonnegative,mustBeReal} % Cost of running a vehicle for 1 km, excluding electricity cost
        BatteryDepreciationPerUnitCharge(1,1) double {mustBeNonnegative,mustBeReal} = 0 % TODO: add explanation
        
        sourcerelaxflag(1,1) logical = false % This flag allows each vehicle flow to reduce its sources (and sinks) for a cost
        
        time_step_s(1,1) double {mustBeNonnegative,mustBeReal} %
        charge_unit_j(1,1) double {mustBeNonnegative,mustBeReal}
        v2g_efficiency(1,1) double {mustBeNonnegative,mustBeReal,mustBeLessThanOrEqual(1)} = 1
        
        yalmip_options(1,1) = sdpsettings();
        
        start_date_time(1,1) datetime = datetime('01-Jan-2010 00:00:00');
    end
    
    
    methods (Access = private)
        obj = UpdatePropertiesDependentOnRoadGraph(obj);
        obj = UpdatePropertiesDependentOnSources(obj);
        ValidateConsistencyWithAdjancencyMatrix(obj,M,varargin);
        
        function res = IsValidRoadNode(obj,i)
            res = IsInteger(i) && 1 <= i && i <= obj.N;
        end        
    end
    
    
end