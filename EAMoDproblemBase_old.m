classdef EAMoDproblemBase < handle
    methods
        function obj = EAMoDproblemBase(eamod_spec)
             
            obj.eamod_spec = eamod_spec;
            
            obj.Thor = eamod_spec.Thor;
            
            RoadNetwork = eamod_spec.RoadNetwork;
            InitialConditions = eamod_spec.InitialConditions;
            Passengers = eamod_spec.Passengers;
            Flags = eamod_spec.Flags;
            
            obj.C = RoadNetwork.C;
            
            obj.RoadGraph = RoadNetwork.RoadGraph;
            
            %Clean up road graph.
            for i=1:length(obj.RoadGraph)
                obj.RoadGraph{i}=sort(unique(obj.RoadGraph{i}));
            end
            
            %Nodes in ReverseRoadGraph{i} are such that RoadGraph{ReverseRoadGraph{i}} contains
            %i
            obj.ReverseRoadGraph = cell(size(obj.RoadGraph));
            for i=1:length(obj.RoadGraph)
                for j=obj.RoadGraph{i}
                    obj.ReverseRoadGraph{j} = [obj.ReverseRoadGraph{j} i];
                end
            end
            for i=1:length(obj.ReverseRoadGraph)
                obj.ReverseRoadGraph{i} = sort(unique(obj.ReverseRoadGraph{i}));
            end
            
            obj.N = length(obj.RoadGraph);
            
            obj.Sources = Passengers.Sources;
            obj.Sinks = Passengers.Sinks;
            obj.Flows=Passengers.Flows;
            obj.StartTimes=Passengers.StartTimes;
            
            obj.NumSinks = length(obj.Sinks);
            
            obj.M = obj.NumSinks;
            
            E_temp = 0;
            NumRoadEdges_temp = zeros(obj.N,1);
            for i = 1:obj.N
                NumRoadEdges_temp(i) = length(obj.RoadGraph{i});
                E_temp = E_temp + length(obj.RoadGraph{i});
            end
            obj.E = E_temp;
            obj.NumRoadEdges = NumRoadEdges_temp;
            
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
            
            % Rearranging for sinks
            obj.NumSourcesPerSink = zeros(size(obj.Sinks));
            for i = 1:obj.NumSinks
                obj.NumSourcesPerSink(i) = length(obj.Sources{i});
            end
            obj.CumNumSourcesPerSink = cumsum(obj.NumSourcesPerSink);
            obj.TotNumSources = obj.CumNumSourcesPerSink(end);
            obj.CumNumSourcesPerSink=[0; obj.CumNumSourcesPerSink(1:end-1)];
            
            obj.RoadCap = RoadNetwork.RoadCap;
            
            obj.TravelTimes = RoadNetwork.TravelTimes;
            obj.TravelDistance = RoadNetwork.TravelDistance;
            
            obj.ChargersList = RoadNetwork.ChargersList;
            obj.NumChargers = length(obj.ChargersList);
            obj.ChargerTime = RoadNetwork.ChargerTime;
            
            obj.ChargeToTraverse = RoadNetwork.ChargeToTraverse;
            obj.ChargerSpeed = RoadNetwork.ChargerSpeed;
            obj.ChargerCap = RoadNetwork.ChargerCap;
            
            % Initial conditions
            obj.FullVehicleInitialPos = InitialConditions.FullVehicleInitialPos;
            obj.EmptyVehicleInitialPos = InitialConditions.EmptyVehicleInitialPos;
            
            % Final conditions
            obj.MinEndCharge = RoadNetwork.MinEndCharge;
            
            % Economic settings
            obj.ValueOfTime = RoadNetwork.ValueOfTime;
            obj.VehicleCostPerKm = RoadNetwork.VehicleCostPerKm;
            
            % Relaxation
            if Flags.congrelaxflag
                warning('congrelaxflag is set but functionality is not available yet. Ignoring.')
            end
            
            obj.sourcerelaxflag = Flags.sourcerelaxflag;            
            
            % Stuff for real time algorithm
            obj.TVRoadCap = zeros(obj.Thor,obj.N,obj.N);
            for tt = 1:obj.Thor
                obj.TVRoadCap(tt,:,:) = obj.RoadCap;
            end
            
            [obj.RouteTime,obj.RouteCharge,obj.RouteDistance,obj.Routes] = obj.BuildRoutes();
            
            % Extras
            obj.ChargeUnitToPowerUnit = eamod_spec.ChargeUnitToPowerUnit;
            obj.v2g_efficiency = eamod_spec.v2g_efficiency;
            obj.charge_unit_j = eamod_spec.charge_unit_j;
            obj.time_step_s = eamod_spec.time_step_s;
        end
        
        [f_cost,f_cost_pax,f_cost_reb,f_cost_relax] = CreateCostVector(obj);
        
        [A_eq,B_eq,A_in,B_in] = CreateConstraintMatrices(obj)
        
        [Aeq, Beq] = CreateEqualityConstraintMatrices_ConservationPax(obj);
        
        [Aineq, Bineq] = CreateInequalityConstraintMatrices(obj);
        
        [RouteTime,RouteCharge,RouteDistance,Routes] = BuildRoutes(obj);
        
        % State vector indexing functions
        
        function res = FindRoadLinkHelpertckij(obj,t,c,k,i,j)
            res = (t-1)*(obj.E*(obj.num_passenger_flows + 1)*(obj.C) + 2*(obj.num_passenger_flows+1)*obj.NumChargers*(obj.C)) + (c-1)*obj.E*(obj.num_passenger_flows+1) + (k-1)*obj.E + (obj.cumRoadNeighbors(i) + obj.RoadNeighborCounter(i,j));
        end
        
        function res = FindRoadLinkPtckij(obj,t,c,k,i,j)
            if obj.use_real_time_formulation
                error('FindRoadLinkPtckij is undefined for real time formulation.');
            else
                res = obj.FindRoadLinkHelpertckij(t,c,k,i,j);
            end
        end
        
        function res = FindRoadLinkRtcij(obj,t,c,i,j)
            if obj.use_real_time_formulation
                res = obj.FindRoadLinkHelpertckij(t,c,1,i,j);
            else
                res = obj.FindRoadLinkPtckij(t,c,obj.M + 1,i,j);
            end
        end
        
        function res = FindChargeLinkHelpertckij(obj,t,c,k,i)
            res = obj.FindRoadLinkRtcij(t,obj.C,obj.N,obj.RoadGraph{end}(end)) + obj.NumChargers*(obj.num_passenger_flows + 1)*(c-1) + obj.NumChargers*(k-1) + i;  %Here we index the charger directly (as opposed to the node hosting the charger)
        end
        
        function res = FindChargeLinkPtckl(obj,t,c,k,i)
            if obj.use_real_time_formulation
               error('FindChargeLinkPtckl is undefined for real time formulation.');
            else
               res = obj.FindChargeLinkHelpertckij(t,c,k,i);
            end
        end
        
        function res = FindChargeLinkRtcl(obj,t,c,i)
            if obj.use_real_time_formulation
                res = obj.FindChargeLinkHelpertckij(t,c,1,i);                
            else
                res = obj.FindChargeLinkPtckl(t,c,obj.M+1,i);
            end
        end
        
        function res = FindDischargeLinkHelpertckl(obj,t,c,k,i)
            res = obj.FindChargeLinkRtcl(t,obj.C,obj.NumChargers) + obj.NumChargers*(obj.num_passenger_flows + 1)*(c-1) + obj.NumChargers*(k-1) + i;
        end
        
        function res = FindDischargeLinkPtckl(obj,t,c,k,i)
            if obj.use_real_time_formulation
                error('FindDischargeLinkPtckl is undefined for real time formulation.');
            else
                res = obj.FindDischargeLinkHelpertckl(t,c,k,i);        
            end
        end
        
        function res = FindDischargeLinkRtcl(obj,t,c,i)
            if obj.use_real_time_formulation
                res = obj.FindDischargeLinkHelpertckl(t,c,1,i);    
            else
                res = obj.FindDischargeLinkPtckl(t,c,obj.M + 1,i);
            end
        end
        
        function res = FindPaxSourceChargecks(obj,c,k,s)
            res = obj.FindDischargeLinkRtcl(obj.Thor,obj.C,obj.NumChargers) + obj.TotNumSources*(c-1) + obj.CumNumSourcesPerSink(k) + s;
        end
        
        function res = FindPaxSinkChargetck(obj,t,c,k)
            res = obj.FindPaxSourceChargecks(obj.C,obj.M,obj.NumSourcesPerSink(end)) + obj.C*obj.M*(t-1) + obj.M*(c-1) + k;
        end
                
        function res = FindEndRebLocationci(obj,c,i)
            res = obj.FindPaxSinkChargetck(obj.Thor,obj.C,obj.M) + obj.N*(c-1) + i;
        end
        
        % Constraint Indexers
        function res = FindEqPaxConservationtcki(obj,t,c,k,i)
            res = obj.N*obj.M*obj.C*(t - 1) + obj.N*obj.M*(c - 1) + obj.N*(k - 1) + i;
        end
        
        % This is untested
        function [t,c,k,i] = ReverseFindEqPaxConservationtcki(obj,in)
            [t,c,k,i] = obj.DecomposeNumber(in,obj.C, obj.M, obj.N);
        end
        
        function varargout = DecomposeNumber(obj,in,varargin)
            n_varargin = numel(varargin);
            
            remainder = in;
            
            varargout = cell(n_varargin + 1,1);
            
            for i_argin = 1:n_varargin
                divisor = prod([varargin{i_argin:end}]);
                
                quotient = floor(remainder/divisor);
                
                varargout{i_argin} = quotient + 1;
                
                remainder = remainder - divisor*quotient;
            end
            
            varargout{n_varargin + 1} = remainder;            
        end
        
        
        
        function res = FindEqRebConservationtci(obj,t,c,i)
            res = obj.N*obj.C*(t - 1) + obj.N*(c - 1) + i;
        end
        
        function res = FindEqSourceConservationks(obj,k,s)
            res = obj.CumNumSourcesPerSink(k) + s;
        end
        
        function res = FindEqSinkConservationk(obj,k)
            res = k;
        end
        
        function res = FindInRoadCongestiontij(obj,t,i,j)
            res = obj.E*(t - 1) + obj.cumRoadNeighbors(i) + obj.RoadNeighborCounter(i,j);
        end
        
        function res = FindInChargerCongestiontl(obj,t,l)
            res = obj.NumChargers*(t-1) + l;
        end
        
        % The indexing for FindSourceRelaxks is different from
        % TVPowerBalancedFlowFinder_sinkbundle because we do not include
        % the power network and we do not include the congestion relaxation
        function res = FindSourceRelaxks(obj,k,s)
            if obj.sourcerelaxflag
                res = obj.FindEndRebLocationci(obj.C,obj.N) +  obj.CumNumSourcesPerSink(k) + s;
            else
                res = nan;
            end
        end
        
        
        function res = get.StateSize(obj)
            if obj.sourcerelaxflag
                res = obj.FindSourceRelaxks(obj.NumSinks,obj.NumSourcesPerSink(obj.NumSinks));
            else
                res = obj.FindEndRebLocationci(obj.C,obj.N);
            end            
        end
        
        function res = get.num_passenger_flows(obj)
            if obj.use_real_time_formulation
                res = 0;
            else
                res = obj.M; 
            end
        end
    end
    
    properties (Dependent)
        StateSize       
        num_passenger_flows
    end
    
    
    
    properties
        Thor
        
        % Road network properties
        N
        
        TravelTimes
        TravelDistance
        
        ChargersList
        ChargerTime
        ChargerSpeed
        ChargerCap
        
        NumChargers
        
        RoadGraph
        ReverseRoadGraph
        NumRoadEdges
        ChargeToTraverse
        
        
        cumRoadNeighbors
        RoadNeighborCounter
        
        RoadCap
        
        % Extended network properties
        C
        E
        
        % Passenger properties
        M
        
        Sources
        Sinks
        Flows
        StartTimes
        
        
        NumSourcesPerSink
        CumNumSourcesPerSink
        TotNumSources
        
        NumSinks
        
        
        
        % Economic settings
        ValueOfTime
        VehicleCostPerKm
        
        % Initial conditions
        FullVehicleInitialPos
        EmptyVehicleInitialPos
        
        % Final conditions
        MinEndCharge
        
        % Relaxation
        sourcerelaxflag
        SourceRelaxCost
        
        
        
        % Stuff for real time algorithm
        use_real_time_formulation(1,1) logical = false;
        TVRoadCap
        RouteTime
        RouteCharge
        RouteDistance
        Routes
        
        ChargeUnitToPowerUnit
        v2g_efficiency
        charge_unit_j
        time_step_s
        
        eamod_spec
    end
    
    
    
end