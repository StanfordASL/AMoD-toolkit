classdef EAMoDspec
    methods
        function obj = EAMoDspec()
        end
        
        function ValidateSpec(obj)
            % ValidateSpec asserts that the properties of the spec are
            % consistent with eachother.
            
            % Properties that depend on RoadGraph
            obj.ValidateConsistencyWithAdjancencyMatrix(obj.RoadCap,'RoadCap');
            obj.ValidateConsistencyWithAdjancencyMatrix(obj.TravelTimes,'TravelTimes');
            obj.ValidateConsistencyWithAdjancencyMatrix(obj.TravelDistance,'TravelDistance');
            obj.ValidateConsistencyWithAdjancencyMatrix(obj.ChargeToTraverse,'ChargeToTraverse');
            
            % Properties that depend on ChargersList
            assert(numel(obj.ChargerSpeed) == obj.NumChargers,'ChargerSpeed must have NumChargers elements.');
            assert(numel(obj.ChargerTime) == obj.NumChargers,'ChargerTime must have NumChargers elements');
            assert(numel(obj.ChargerTime) == obj.NumChargers,'ChargerCap must have NumChargers elements');
            
            % Properties that depend on Sinks
            assert(numel(obj.Sources) == obj.NumSinks,'Sources must have NumSinks elements');
            
            % Properties that depend on Sources
            assert(all(cellfun(@numel,obj.StartTimes) == obj.NumSourcesPerSink),'StartTimes must have the same number of cells and number of elements per cell as Sources');
            assert(all(cellfun(@numel,obj.Flows) == obj.NumSourcesPerSink),'Flows must have the same number of cells and number of elements per cell as Sources');
            
            % Entries that must be valid road nodes
            assert(all(obj.IsValidRoadNode(obj.ChargersList)),'Entries in ChargersList must be valid road nodes');
            assert(all(obj.IsValidRoadNode(obj.Sinks)),'Entries in Sinks must be valid road nodes');
            
            for i_sink = 1:obj.NumSinks
                sources_towards_sink = obj.Sources{i_sink};
                assert(all(obj.IsValidRoadNode(sources_towards_sink)),'Entries in Sources{%i} must be valid road nodes',i_sink);
            end
            
            % Others
            assert(obj.MinEndCharge <= obj.C - 1,'MinEndCharge may not be more than C - 1 (recall it is exclusive)');
            
            assert(all(size(obj.FullVehicleInitialPos) == [obj.M,obj.N,obj.C]),'FullVehicleInitialPos must be of size M x N x C');
            assert(all(size(obj.EmptyVehicleInitialPos) == [obj.N,obj.C]),'FullVehicleInitialPos must be of size N x C');
            
        end       
       
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
        function ValidateRoadGraph(RoadGraph)
            % ValidateRoadGraph asserts that the neighbor lists in
            % RoadGraph point to nodes that are integer and in [0,N].
            
            N = numel(obj.RoadGraph);
            
            for i = 1:numel(obj.RoadGraph)
                neighbor_vec = RoadGraph{i};                
                assert(all(obj.IsValidRoadNode(neighbor_vec)),'Not all neighbors of node %i are valid road nodes',i)
            end
        end
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
        function obj = UpdatePropertiesDependentOnRoadGraph(obj)
            obj.N = numel(obj.RoadGraph);
            
            obj.ReverseRoadGraph = cell(size(obj.RoadGraph));
            for i=1:obj.N
                for j=obj.RoadGraph{i}
                    obj.ReverseRoadGraph{j} = [obj.ReverseRoadGraph{j} i];
                end
            end
            for i=1:numel(obj.ReverseRoadGraph)
                obj.ReverseRoadGraph{i} = sort(unique(obj.ReverseRoadGraph{i}));
            end
            
            obj.adjacency_matrix = false(obj.N,obj.N);
            for i = 1:obj.N
                for neighbor_c = obj.RoadGraph{i}
                    obj.adjacency_matrix(i,neighbor_c) = 1;
                end
            end
            
            obj.E = 0;
            obj.NumRoadEdges = zeros(obj.N,1);
            for i = 1:obj.N
                obj.NumRoadEdges(i) = length(obj.RoadGraph{i});
                obj.E = obj.E + length(obj.RoadGraph{i});
            end
            
            obj.cumRoadNeighbors = cumsum(obj.NumRoadEdges);
            obj.cumRoadNeighbors=[0;obj.cumRoadNeighbors(1:end-1)];
            
            obj.RoadNeighborCounter = sparse([],[],[],obj.N,obj.N,obj.E);
            TempNeighVec = zeros(obj.N,1);
            for i = 1:obj.N
                for j = obj.RoadGraph{i}
                    TempNeighVec(j) = 1;
                end
                NeighCounterLine = cumsum(TempNeighVec);
                for j = obj.RoadGraph{i}
                    obj.RoadNeighborCounter(i,j) = NeighCounterLine(j);
                end
                TempNeighVec = zeros(obj.N,1);
            end
        end
        
        function obj = UpdatePropertiesDependentOnSources(obj)
            obj.NumSourcesPerSink = cellfun(@numel,obj.Sources);
            
            obj.CumNumSourcesPerSink = cumsum(obj.NumSourcesPerSink);
            obj.TotNumSources = obj.CumNumSourcesPerSink(end);
            obj.CumNumSourcesPerSink=[0; obj.CumNumSourcesPerSink(1:end-1)];            
        end
        
        function ValidateConsistencyWithAdjancencyMatrix(obj,M,varargin)
            % ValidateConsistencyWithAdjancencyMatrix asserts that a matrix M
            % has the same size as adjacency_matrix and that M is zero when
            % adjacency_matrix is zero.
            % ValidateConsistencyWithAdjancencyMatrix(obj,M)
            % ValidateConsistencyWithAdjancencyMatrix(obj,M,M_name) where
            % M_name is the name of M to be included in error messages
            
            n_varargin = numel(varargin);
            
            switch n_varargin
                case 0
                    M_name = 'M';
                case 1
                    M_name = varargin{1};
                    
                otherwise
                    error('Too many arguments')
            end
            
            assert(all(size(M) == size(obj.adjacency_matrix)),'Size of %s must match that of adjacency_matrix.',M_name)
            assert(all(M(obj.adjacency_matrix) == 0),'%s must be zero when adjacency_matrix is zero.',M_name)
        end
        
        function res = IsValidRoadNode(obj,i)
            res = IsInteger(i) && 1 <= i && i <= obj.N;
        end
        
    end
    
    
end