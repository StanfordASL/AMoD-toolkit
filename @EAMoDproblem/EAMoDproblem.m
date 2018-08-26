classdef EAMoDproblem < handle
    % EAMoDproblem Represents an electric AMoD problem using a network flow model
    %   The model of the electric AMoD system used here is described in the
    %   paper below. Note that this implementation does not include the power network.
    %
    %   F. Rossi, R. Iglesias, n_passenger_flow. Alizadeh, and n_passenger_flow. Pavone, “On the interaction
    %   between Autonomous Mobility-on-Demand systems and the power network:
    %   models and coordination algorithms,” in Robotics: Science and Systems,
    %   Pittsburgh, Pennsylvania, 2018
    
    methods
        function obj = EAMoDproblem(spec)
            % EAMoDproblem Constructs an EAMoDproblem object
            %   obj = EAMoDproblem(spec) where spec is an instance of
            %   EAMoDspec specifying the problem.
            %   See also EAMoDspec
            
            validateattributes(spec,{'EAMoDspec'},{'scalar'},mfilename,'spec',1);
            
            spec.ValidateSpec();
            obj.spec = spec;
            
            [obj.RouteTime,obj.RouteCharge,obj.RouteDistance,obj.Routes] = obj.BuildRoutes();
            obj.TVRoadCap = obj.ComputeResidualRoadCapacity();
            
            
        end
                
        n_start_vehicles = ComputeNumberOfVehiclesAtStart(obj)
        n_end_vehicles = ComputeNumberOfVehiclesAtEnd(obj,varargin)
        [total_cost_val, pax_cost_val, reb_cost_val,relax_cost_val] = EvaluateAMoDcost(obj,varargin)
        charger_power_demand_val_w = EvaluateChargerPowerDemand(obj,varargin)
        decision_vector_val = EvaluateDecisionVector(obj);    
        electricity_cost_val_usd = EvaluateElectricityCost(obj,varargin)    
        final_vehicle_distribution = GetFinalVehicleDistribution(obj,varargin)
        [DepTimeHist, ArrivalTimeHist] = GetTravelTimesHistograms(obj,varargin);
        [ChargingVehicleHist,DischargingVehicleHist,PaxVehicleHist,RebVehicleHist,IdleVehicleHist,AllVehicleHist] = GetVehicleStateHistograms(obj,varargin)
        [objective_value,solver_time,diagnostics] = Solve(obj)
                
        % Ploting methods
        figure_handle = PlotRoadGraph(obj)
        figure_handle = PlotDeparturesAndArrivals(obj,varargin)
        figure_handle = PlotVehicleState(obj,params_plot,varargin)
        
        % Determine matrices for the linear program
        [f_cost,f_cost_pax,f_cost_reb,f_cost_relax] = CreateCostVector(obj)
        
        % Equality constraints for standard formulation
        [Aeq_PaxConservation, Beq_PaxConservation] = CreateEqualityConstraintMatrices_PaxConservation(obj)
        [Aeq_RebConservation, Beq_RebConservation] = CreateEqualityConstraintMatrices_RebConservation(obj)
        [Aeq_SourceConservation, Beq_SourceConservation] = CreateEqualityConstraintMatrices_SourceConservation(obj)
        [Aeq_SinkConservation, Beq_SinkConservation] = CreateEqualityConstraintMatrices_SinkConservation(obj)
        
        % Equality constraints for real-time formulation
        [Aeq_CustomerChargeConservation, Beq_CustomerChargeConservation] = CreateEqualityConstraintMatrices_CustomerChargeConservation(obj)
        
        % Inequality constraints
        [Ain_RoadCongestion, Bin_RoadCongestion] = CreateInequalityConstraintMatrices_RoadCongestion(obj)
        [Ain_ChargerCongestion, Bin_ChargerCongestion] = CreateInequalityConstraintMatrices_ChargerCongestion(obj)
        
        [lb_StateVector,ub_StateVector] = CreateStateVectorBounds(obj)
                            
        % State vector indexing functions
        function res = FindRoadLinkPtckij(obj,t,c,k,i,j)
            % FindRoadLinkPtckij Indexer for customer flow in road edges in the extended network
            if obj.use_real_time_formulation
                error('FindRoadLinkPtckij is undefined for real time formulation.');
            else
                res = obj.FindRoadLinkHelpertckij(t,c,k,i,j);
            end
        end
        
        function res = FindRoadLinkRtcij(obj,t,c,i,j)
            % FindRoadLinkRtcij Indexer for rebalancing flow in road edges in the extended network
            if obj.use_real_time_formulation
                res = obj.FindRoadLinkHelpertckij(t,c,1,i,j);
            else
                res = obj.FindRoadLinkPtckij(t,c,obj.spec.n_passenger_flow + 1,i,j);
            end
        end
        
        function res = FindChargeLinkPtckl(obj,t,c,k,i)
            % FindChargeLinkPtckl Indexer for customer flow in charging edges in the extended network
            if obj.use_real_time_formulation
                error('FindChargeLinkPtckl is undefined for real time formulation.');
            else
                res = obj.FindChargeLinkHelpertckij(t,c,k,i);
            end
        end
        
        function res = FindChargeLinkRtcl(obj,t,c,i)
            % FindChargeLinkRtcl Indexer for rebalancing flow in charging edges in the extended network
            if obj.use_real_time_formulation
                res = obj.FindChargeLinkHelpertckij(t,c,1,i);
            else
                res = obj.FindChargeLinkPtckl(t,c,obj.spec.n_passenger_flow+1,i);
            end
        end
        
        function res = FindDischargeLinkPtckl(obj,t,c,k,i)
            % FindDischargeLinkPtckl Indexer for customer flow in discharging edges in the extended network
            if obj.use_real_time_formulation
                error('FindDischargeLinkPtckl is undefined for real time formulation.');
            else
                res = obj.FindDischargeLinkHelpertckl(t,c,k,i);
            end
        end
        
        function res = FindDischargeLinkRtcl(obj,t,c,i)
            % FindDischargeLinkRtcl Indexer for rebalancing flow in discharging edges in the extended network
            if obj.use_real_time_formulation
                res = obj.FindDischargeLinkHelpertckl(t,c,1,i);
            else
                res = obj.FindDischargeLinkPtckl(t,c,obj.spec.n_passenger_flow + 1,i);
            end
        end
        
        function res = FindPaxSourceChargecks(obj,c,k,s)
            % FindPaxSourceChargecks Indexer for sources at a given charge c going from source s to sink k
            res = obj.FindDischargeLinkRtcl(obj.spec.n_time_step,obj.spec.n_charge_step,obj.spec.n_charger) + obj.spec.n_passenger_source*(c-1) + obj.spec.n_sources_to_sink_cumsum(k) + s;
        end
        
        function res = FindPaxSinkChargetck(obj,t,c,k)
            % FindPaxSinkChargetck Indexer for sinks k at a given charge c arriving at time t
            res = obj.FindPaxSourceChargecks(obj.spec.n_charge_step,obj.spec.n_passenger_flow,obj.spec.n_sources_to_sink(end)) + obj.spec.n_charge_step*obj.spec.n_passenger_flow*(t-1) + obj.spec.n_passenger_flow*(c-1) + k;
        end
        
        function res = FindEndRebLocationci(obj,c,i)
            % FindEndRebLocationci Indexer for final position of rebalancing vehicles
            res = obj.FindPaxSinkChargetck(obj.spec.n_time_step,obj.spec.n_charge_step,obj.spec.n_passenger_flow) + obj.spec.n_road_node*(c-1) + i;
        end
        
        % Constraint Indexers
        function res = FindEqPaxConservationtcki(obj,t,c,k,i)
            % FindEqPaxConservationtcki Indexer for customer flow conservation constraints
            res = obj.spec.n_road_node*obj.spec.n_passenger_flow*obj.spec.n_charge_step*(t - 1) + obj.spec.n_road_node*obj.spec.n_passenger_flow*(c - 1) + obj.spec.n_road_node*(k - 1) + i;
        end
        
        function res = FindEqRebConservationtci(obj,t,c,i)
            % FindEqRebConservationtci Indexer for rebalancing flow conservation constraints
            res = obj.spec.n_road_node*obj.spec.n_charge_step*(t - 1) + obj.spec.n_road_node*(c - 1) + i;
        end
        
        function res = FindEqSourceConservationks(obj,k,s)
            % FindEqSourceConservationks Indexer for source conservation constraints
            res = obj.spec.n_sources_to_sink_cumsum(k) + s;
        end
        
        function res = FindEqSinkConservationk(obj,k)
            % FindEqSinkConservationk Indexer for sink conservation constraints
            res = k;
        end
        
        function res = FindInRoadCongestiontij(obj,t,i,j)
            % FindInRoadCongestiontij Indexer for road congestion constraints
            res = obj.spec.n_road_edge*(t - 1) + obj.spec.edge_number_matrix(i,j);
        end
        
        function res = FindInChargerCongestiontl(obj,t,l)
            % FindInChargerCongestiontl Indexer for charger congestion constraints
            res = obj.spec.n_charger*(t-1) + l;
        end
        
        % The indexing for FindSourceRelaxks is different from
        % TVPowerBalancedFlowFinder_sinkbundle because we do not include
        % the power network and we do not include the congestion relaxation
        function res = FindSourceRelaxks(obj,k,s)
            % FindSourceRelaxks Indexer for constraints relaxing sources
            if obj.sourcerelaxflag
                res = obj.FindEndRebLocationci(obj.spec.n_charge_step,obj.spec.n_road_node) +  obj.spec.n_sources_to_sink_cumsum(k) + s;
            else
                res = nan;
            end
        end
        
        function res = FindChargerPowertl(obj,t,l)
            % FindSourceRelaxks Indexer for row in ComputeChargerPowerMatrixNew corresponding to charger l at time step t  
            res = obj.spec.n_charger*(t - 1) + l;
        end
        
        % Get methods for dependent properties
        function res = get.StateSize(obj)
            if obj.sourcerelaxflag
                res = obj.FindSourceRelaxks(obj.spec.n_passenger_sink,obj.spec.n_sources_to_sink(obj.spec.n_passenger_sink));
            else
                res = obj.FindEndRebLocationci(obj.spec.n_charge_step,obj.spec.n_road_node);
            end
        end
        
        function res = get.num_passenger_flows(obj)
            if obj.use_real_time_formulation
                res = 0;
            else
                res = obj.spec.n_passenger_flow;
            end
        end
        
        function res = get.state_range(obj)
            res = 1:obj.FindEndRebLocationci(obj.spec.n_charge_step,obj.spec.n_road_node);
        end
        
        function res = get.relax_range(obj)
            res = (obj.state_range(end) + 1):obj.StateSize;
        end
    end
    
    properties (Dependent)
        StateSize % Number of elements in the problem's state vector
        num_passenger_flows % Number of passenger flows. Is equal to spec.n_passenger_flow in the normal case and zero in the real-time formulation
    
        % TODO: add description
        state_range(1,:) double
        relax_range(1,:) double
    end
    
    properties       
        use_real_time_formulation(1,1) logical = false % Flag to use real-time formulation
        
        sourcerelaxflag(1,1) logical = false % Flag to allow each vehicle flow to reduce its sources and sinks for cost SourceRelaxCost        
        SourceRelaxCost(1,1) double {mustBeNonnegative,mustBeReal} % Cost for relaxing sources and sinks (only active when sourcerelaxflag is set)
                       
        verbose(1,1) logical = false % Flag for verbose output
        
        yalmip_settings(1,1) = sdpsettings() % Struct with YALMIP settings
    end
    
    properties (SetAccess = private, GetAccess = public)
        spec(1,1) EAMoDspec % Object of class EAMoDspec specifying the problem
                
        % For real-time formulation
        
        RouteTime(:,:)  double {mustBeNonnegative,mustBeReal,mustBeInteger}  % RouteTime(i,j) is the number of time-steps needed to go from i to j
        RouteCharge(:,:)  double {mustBeNonnegative,mustBeReal,mustBeInteger} % RouteCharge(i,j) is the number of charge units needed to go from i to j
        RouteDistance(:,:)  double {mustBeNonnegative,mustBeReal} % RouteDistance(i,j) is the distance in meters to go from i to j
        Routes(:,:) cell % Routes{i,j} is the route from i to j expressed as a vector of connected nodes that need to be traversed
        
        TVRoadCap(:,:,:) double {mustBeReal} % TVRoadCap(i,j,t) is the residual capacity of the i-j link (in vehicles per unit time) at time t, after subtracting the flow from pre-routed vehicles     
    end
    
    properties (Access = private)
        % TODO: rename to optimization_variables
        decision_variables(1,1) % Struct with optimization variables        
    end
    
    methods (Access = private)
        [RouteTime,RouteCharge,RouteDistance,Routes] = BuildRoutes(obj)          
        charger_power_demand_w = ComputeChargerPowerDemand(obj,varargin)
        charger_power_demand = ComputeChargerPowerDemandNormalized(obj,factor,varargin)
        A_charger_power_w = ComputeChargerPowerMatrixNew(obj)         
        pax_cost_rt = ComputePaxCostRealTimeFormulation(obj)
        TVRoadCap = ComputeResidualRoadCapacity(obj)
        decision_variables = DefineDecisionVariables(obj)
        [amod_cost_usd, pax_cost_usd,reb_cost_usd, relax_cost_usd] = ComputeAMoDcost(obj,varargin)
        electricity_cost_usd = ComputeElectricityCost(obj,varargin)
        constraint_array = GetConstraintArray(obj)
        objective = GetObjective(obj)
        pre_routed_trip_histogram = GetPreRoutedTripHistogram(obj)
        
        function res = FindRoadLinkHelpertckij(obj,t,c,k,i,j)
            res = (t-1)*(obj.spec.n_road_edge*(obj.num_passenger_flows + 1)*(obj.spec.n_charge_step) + 2*(obj.num_passenger_flows+1)*obj.spec.n_charger*(obj.spec.n_charge_step)) + (c-1)*obj.spec.n_road_edge*(obj.num_passenger_flows+1) + (k-1)*obj.spec.n_road_edge + obj.spec.edge_number_matrix(i,j);
        end
        
        function res = FindChargeLinkHelpertckij(obj,t,c,k,i)
            res = obj.FindRoadLinkRtcij(t,obj.spec.n_charge_step,obj.spec.n_road_node,obj.spec.road_adjacency_list{end}(end)) + obj.spec.n_charger*(obj.num_passenger_flows + 1)*(c-1) + obj.spec.n_charger*(k-1) + i;  %Here we index the charger directly (as opposed to the node hosting the charger)
        end
        
        function res = FindDischargeLinkHelpertckl(obj,t,c,k,i)
            res = obj.FindChargeLinkRtcl(t,obj.spec.n_charge_step,obj.spec.n_charger) + obj.spec.n_charger*(obj.num_passenger_flows + 1)*(c-1) + obj.spec.n_charger*(k-1) + i;
        end
    end
end