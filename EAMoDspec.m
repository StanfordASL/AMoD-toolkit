classdef EAMoDspec
    methods
        function obj = EAMoDspec(Thor,C,RoadGraph)
            
            validateattributes(RoadGraph,{'cell'},{},mfilename,'RoadGraph',1);
            
            obj.N = numel(obj.RoadGraph);
            
            validateattributes(RoadGraph,{'cell'},{},mfilename,'Sources',3);
            
            % Clean up road graph.
            for i = 1:length(obj.RoadGraph)
                RoadGraph{i} = sort(unique(RoadGraph{i}));
            end
            
            EAMoDspec.ValidateRoadGraph(RoadGraph);
            obj.RoadGraph = RoadGraph(:);
            
            
            
            obj.adjacency_matrix = false(obj.N,obj.N);
            
            for i = 1:obj.N
                for neighbor_c = obj.RoadGraph{i}
                    obj.adjacency_matrix(i,neighbor_c) = 1;
                end
            end
        end
        
        % It is sufficient to look at the initial conditions because
        % vehicles are conserved
        function n_vehicle = get.n_vehicle(obj)
            n_vehicle_full_init = sum(obj.InitialConditions.FullVehicleInitialPos(:));
            n_vehicle_empty_init = sum(obj.InitialConditions.EmptyVehicleInitialPos(:));
            
            n_vehicle = n_vehicle_full_init + n_vehicle_empty_init;
        end
        
        % This is for convenience and compatibility with other specs
        function n_time_step = get.n_time_step(obj)
            n_time_step = obj.Thor;
        end
        
        %         function obj = set.RoadCap(obj,RoadCap)
        %             ValidateConsistencyWithAdjancencyMatrix(obj.adjacency_matrix,RoadCap);
        %             obj.RoadCap = RoadCap;
        %         end
        %
        %         function obj = set.TravelTimes(obj,TravelTimes)
        %             ValidateConsistencyWithAdjancencyMatrix(obj.adjacency_matrix,TravelTimes);
        %             obj.TravelTimes = TravelTimes;
        %         end
        %
        %         function obj = set.TravelDistance(obj,TravelDistance)
        %             ValidateConsistencyWithAdjancencyMatrix(obj.adjacency_matrix,TravelDistance);
        %             obj.TravelDistance = TravelDistance;
        %         end
        %
        %         function obj = set.ChargeToTraverse(obj,ChargeToTraverse)
        %             ValidateConsistencyWithAdjancencyMatrix(obj.adjacency_matrix,ChargeToTraverse);
        %             obj.ChargeToTraverse = ChargeToTraverse;
        %         end
        %
        %         function obj = set.ChargerSpeed(obj,ChargerSpeed)
        %             assert(numel(ChargerSpeed) == obj.n_charger)
        %             obj.ChargerSpeed = ChargerSpeed;
        %         end
        %
        %         function obj = set.ChargerSpeed(obj,ChargerSpeed)
        %             assert(numel(ChargerSpeed) == obj.n_charger)
        %             obj.ChargerSpeed = ChargerSpeed;
        %         end
        
    end
    
    methods (Static)
        function ValidateRoadGraph(RoadGraph)
            % ValidateRoadGraph asserts that the neighbor lists in
            % RoadGraph point to nodes that are integer and in [0,N].
            
            N = numel(obj.RoadGraph);
            
            for i = 1:numel(obj.RoadGraph)
                neighbor_vec = RoadGraph{i};
                
                assert(all(round(neighbor_vec) == neighbor_vec),'Neighbors of node %i are not integers',i);
                assert(all(1 <= neighbor_vec & neighbor_vec <= N),'Neighbors of node %i are not in range [0,N]',i);
            end
        end
        
        function ValidateConsistencyWithAdjancencyMatrix(adjacency_matrix,M)
            % ValidateConsistencyWithAdjancencyMatrix asserts that matrix M
            % has the same size as adjacency_matrix and that M is zero when
            % adjacency_matrix is zero.
            assert(all(size(M) == size(adjacency_matrix)),'Size of M must match that of adjacency_matrix.')
            assert(all(M(adjacency_matrix) == 0),'M must be zero when adjacency_matrix is zero.')
        end
        
        
    end
    
    properties (SetAccess = immutable, GetAccess = public)
        RoadGraph
        N
        adjacency_matrix
        
        n_charger
    end
    
    
    
    
    
    properties
        Thor
        C
        
        % Road network
        RoadCap(:,:) double {mustBeNonnegative,mustBeReal} % RoadCap(i,j) is the capacity of the i-j link (in vehicles per unit time)
        TravelTimes(:,:) double {mustBeNonnegative,mustBeReal,mustBeInteger} % TravelTimes(i,j) is the travel time along the i-j link
        TravelDistance(:,:) double {mustBeNonnegative,mustBeReal} % TravelDistance(i,j) is the travel distance along the i-j link.
        ChargeToTraverse(:,:) double {mustBeNonnegative,mustBeReal,mustBeInteger} % ChargeReqs(i,j) is the amount of units of charge required to travel from i to j
        
        % Charger data
        ChargersList(:,1) double {mustBeNonnegative,mustBeReal,mustBeInteger} % ChargersList(i) is the location of the node in RoadGraph corresponding to the i-th charging station
        ChargerSpeed(:,1) double {mustBeNonnegative,mustBeReal} % ChargerSpeed(i) is the amount of charge gained by a vehicle crossing charging station i
        ChargerTime(:,1) double {mustBeNonnegative,mustBeReal} % ChargerTime(i) is the time required to charge ChargerSpeed(i) units of charge at charging station i
        ChargerCaps(:,1) double {mustBeNonnegative,mustBeReal} % ChargerCaps(i) is the number of vehicles that can charge concurrently in charging station i
        % TODO: verify that size matches number of chargers
        
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
        
        sourcerelaxflag(1,1) logical = false % This flag allows each vehicle flow to reduce its sources (and sinks) for a cost
        
        time_step_s(1,1) double {mustBeNonnegative,mustBeReal}
        charge_unit_j(1,1) double {mustBeNonnegative,mustBeReal}
        v2g_efficiency(1,1) double {mustBeNonnegative,mustBeReal,mustBeLessThanOrEqual(1)}
        
        yalmip_options
        
        start_date_time(1,1) datetime = datetime('01-Jan-2010 00:00:00');
    end
    
    properties (Dependent)
        n_vehicle
        n_time_step
    end
    
    
end