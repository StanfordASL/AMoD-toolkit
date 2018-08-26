classdef (Abstract) AbstractEAMoDproblem < handle
    % AbstractEAMoDproblem Represents an electric AMoD problem using a network flow model
    %   The model of the electric AMoD system used here is described in the
    %   paper below. Note that this implementation does not include the power network.
    %
    %   F. Rossi, R. Iglesias, n_passenger_flow. Alizadeh, and n_passenger_flow. Pavone, “On the interaction
    %   between Autonomous Mobility-on-Demand systems and the power network:
    %   models and coordination algorithms,” in Robotics: Science and Systems,
    %   Pittsburgh, Pennsylvania, 2018
    %
    %   See also EAMoDspec
    
    methods
        function obj = AbstractEAMoDproblem(spec,n_passenger_flow_in_optimization)
            % EAMoDproblem Constructs an EAMoDproblem object
            %   obj = EAMoDproblem(spec) where spec is an instance of
            %   EAMoDspec specifying the problem.
            %
            %   See also EAMoDspec
            
            validateattributes(spec,{'EAMoDspec'},{'scalar'},mfilename,'spec',1);
            
            spec.ValidateSpec();
            obj.spec = spec;
            
            obj.n_passenger_flow_in_optimization = n_passenger_flow_in_optimization;
                        
            obj.road_residual_capacity_matrix = obj.ComputeResidualRoadCapacity();
            
            obj.SetOffsetsForIndexingFunctions();
        end
                
        n_start_vehicles = ComputeNumberOfVehiclesAtStart(obj)
        n_end_vehicles = ComputeNumberOfVehiclesAtEnd(obj,varargin)
        [total_cost_val, pax_cost_val, reb_cost_val,relax_cost_val] = EvaluateAMoDcost(obj,varargin)
        charger_power_demand_val_w = EvaluateChargerPowerDemand(obj,varargin)
        state_vector_val = EvaluateStateVector(obj)   
        electricity_cost_val_usd = EvaluateElectricityCost(obj,varargin)    
        final_vehicle_distribution = GetFinalVehicleDistribution(obj,varargin)
        [DepTimeHist, ArrivalTimeHist] = GetTravelTimesHistograms(obj,varargin);
        [ChargingVehicleHist,DischargingVehicleHist,PaxVehicleHist,RebVehicleHist,IdleVehicleHist,AllVehicleHist] = GetVehicleStateHistograms(obj,varargin)
        [objective_value,solver_time_s,diagnostics] = Solve(obj)
                
        % Plotting methods
        figure_handle = PlotRoadGraph(obj)
        figure_handle = PlotDeparturesAndArrivals(obj,varargin)
        figure_handle = PlotVehicleState(obj,params_plot,varargin)
        
        % Determine matrices for the linear program
        [f_cost,f_cost_pax,f_cost_reb,f_cost_relax] = CreateCostVector(obj)
        
        % Equality constraints
        [Aeq_RebConservation, Beq_RebConservation] = CreateEqualityConstraintMatrices_RebConservation(obj)
        [Aeq_SourceConservation, Beq_SourceConservation] = CreateEqualityConstraintMatrices_SourceConservation(obj)
        [Aeq_SinkConservation, Beq_SinkConservation] = CreateEqualityConstraintMatrices_SinkConservation(obj)
        
        % Inequality constraints
        [Ain_RoadCongestion, Bin_RoadCongestion] = CreateInequalityConstraintMatrices_RoadCongestion(obj)
        [Ain_ChargerCongestion, Bin_ChargerCongestion] = CreateInequalityConstraintMatrices_ChargerCongestion(obj)
        
        [lb_StateVector,ub_StateVector] = CreateStateVectorBounds(obj)
    end
    
    properties        
        source_relax_flag(1,1) logical = false % Flag to allow each vehicle flow to reduce its sources and sinks for cost source_relax_cost        
        source_relax_cost(1,1) double {mustBeNonnegative,mustBeReal} % Cost for relaxing sources and sinks (only relevant when source_relax_flag is set)
                       
        verbose(1,1) logical = false % Flag for verbose output
        
        yalmip_settings(1,1) = sdpsettings() % Struct with YALMIP settings
    end
    
    properties (SetAccess = private, GetAccess = public)
        spec(1,1) EAMoDspec % Object of class EAMoDspec specifying the problem        
    end
    
    properties (Dependent)
        n_state_vector % Number of elements in the problem's state vector
    end
    
    methods                            
        % State vector indexing functions     
        
        function res = FindRoadLinkRtcij(obj,t,c,i,j)
            % FindRoadLinkRtcij Indexer for rebalancing flow in road edges in the extended network 
            
            res = obj.FindRoadLinkHelpertckij(t,c,obj.n_passenger_flow_in_optimization + 1,i,j);
        end
        
        function res = FindChargeLinkRtcl(obj,t,c,i)
            % FindChargeLinkRtcl Indexer for rebalancing flow in charging edges in the extended network
            
            res = obj.FindChargeLinkHelpertckij(t,c,obj.n_passenger_flow_in_optimization + 1,i);
        end
              
        function res = FindDischargeLinkRtcl(obj,t,c,i)
            % FindDischargeLinkRtcl Indexer for rebalancing flow in discharging edges in the extended network
            
            res = obj.FindDischargeLinkHelpertckl(t,c,obj.n_passenger_flow_in_optimization + 1,i);
        end
        
        function res = FindPaxSourceChargecks(obj,c,k,s)
            % FindPaxSourceChargecks Indexer for sources at a given charge c going from source s to sink k
            
            % Original defintion
            % res = obj.FindDischargeLinkRtcl(obj.spec.n_time_step,obj.spec.n_charge_step,obj.spec.n_charger) + obj.spec.n_passenger_source*(c-1) + obj.spec.n_sources_to_sink_cumsum(k) + s;
            % We cache the call to FindDischargeLinkRtcl
            res = obj.offset_FindPaxSourceChargecks + obj.spec.n_passenger_source*(c-1) + obj.spec.n_sources_to_sink_cumsum(k) + s;        
        end
        
        function res = FindPaxSinkChargetck(obj,t,c,k)
            % FindPaxSinkChargetck Indexer for sinks k at a given charge c arriving at time t
            
            % Original defintion
            % res = obj.FindPaxSourceChargecks(obj.spec.n_charge_step,obj.spec.n_passenger_flow,obj.spec.n_sources_to_sink(end)) + obj.spec.n_charge_step*obj.spec.n_passenger_flow*(t-1) + obj.spec.n_passenger_flow*(c-1) + k;
            % We cache the call to FindPaxSourceChargecks
            res = obj.offset_FindPaxSinkChargetck + obj.spec.n_charge_step*obj.spec.n_passenger_flow*(t-1) + obj.spec.n_passenger_flow*(c-1) + k;      
        end
        
        function res = FindEndRebLocationci(obj,c,i)
            % FindEndRebLocationci Indexer for final position of rebalancing vehicles
            
            % Original defintion
            % res = obj.FindPaxSinkChargetck(obj.spec.n_time_step,obj.spec.n_charge_step,obj.spec.n_passenger_flow) + obj.spec.n_road_node*(c-1) + i;
            % We cache the call to FindPaxSinkChargetck            
            res = obj.offset_FindEndRebLocationci + obj.spec.n_road_node*(c-1) + i;        
        end
        
        % Constraint Indexers
        
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
                
        function res = FindSourceRelaxks(obj,k,s)
            % FindSourceRelaxks Indexer for constraints relaxing sources
            %   The indexing for FindSourceRelaxks is different from
            %   TVPowerBalancedFlowFinder_sinkbundle because we do not include
            %   the power network and we do not include the congestion relaxation
            
            if obj.source_relax_flag
                res = obj.FindEndRebLocationci(obj.spec.n_charge_step,obj.spec.n_road_node) +  obj.spec.n_sources_to_sink_cumsum(k) + s;
            else
                res = nan;
            end
        end
        
        function res = FindChargerPowertl(obj,t,l)
            % FindChargerPowertl Indexer for row in ComputeChargerPowerMatrix corresponding to charger l at time step t  
            
            res = obj.spec.n_charger*(t - 1) + l;
        end
        
        % Get methods for dependent properties
        function res = get.n_state_vector(obj)
            if obj.source_relax_flag
                res = obj.FindSourceRelaxks(obj.spec.n_passenger_sink,obj.spec.n_sources_to_sink(obj.spec.n_passenger_sink));
            else
                res = obj.FindEndRebLocationci(obj.spec.n_charge_step,obj.spec.n_road_node);
            end
        end
    end
    
    methods (Abstract, Access = protected)
        road_residual_capacity_matrix = ComputeResidualRoadCapacity(obj)
        constraint_array = GetConstraintArray(obj)
    end
    
    methods (Access = protected)
        [amod_cost_usd, pax_cost_usd,reb_cost_usd, relax_cost_usd] = ComputeAMoDcost(obj,varargin)
        constraint_array = GetCommonConstraintArray(obj)
        
        function res = FindRoadLinkHelpertckij(obj,t,c,k,i,j)
            % FindRoadLinkHelpertckij Helper for FindRoadLinkPtckij and FindRoadLinkRtcij
            % 
            % See also EAMoDproblem.FindRoadLinkPtckij, AbstractEAMoDproblem.FindRoadLinkRtcij
            
            res = (t-1)*(obj.spec.n_road_edge*(obj.n_passenger_flow_in_optimization + 1)*(obj.spec.n_charge_step) + 2*(obj.n_passenger_flow_in_optimization+1)*obj.spec.n_charger*(obj.spec.n_charge_step)) + (c-1)*obj.spec.n_road_edge*(obj.n_passenger_flow_in_optimization+1) + (k-1)*obj.spec.n_road_edge + obj.spec.edge_number_matrix(i,j);
        end
        
        function res = FindChargeLinkHelpertckij(obj,t,c,k,i)
            % FindChargeLinkHelpertckij Helper for FindChargeLinkPtckl and FindChargeLinkRtcl
            % 
            % See also EAMoDproblem.FindChargeLinkPtckl, AbstractEAMoDproblem.FindChargeLinkRtcl
            
            % Original definition
            % res = obj.FindRoadLinkRtcij(t,obj.spec.n_charge_step,obj.spec.n_road_node,obj.spec.road_adjacency_list{end}(end)) + obj.spec.n_charger*(obj.n_passenger_flow_in_optimization + 1)*(c-1) + obj.spec.n_charger*(k-1) + i;  %Here we index the charger directly (as opposed to the node hosting the charger)
            % We cache the calls to obj.FindRoadLinkRtcij for better performance
            res = obj.offset_FindChargeLinkHelpertckij_t(t) + obj.spec.n_charger*(obj.n_passenger_flow_in_optimization + 1)*(c-1) + obj.spec.n_charger*(k-1) + i;  %Here we index the charger directly (as opposed to the node hosting the charger)
        end
        
        function res = FindDischargeLinkHelpertckl(obj,t,c,k,i)
            % FindDischargeLinkHelpertckl Helper for FindDischargeLinkPtckl and FindDischargeLinkRtcl
            % 
            % See also EAMoDproblem.FindDischargeLinkPtckl, AbstractEAMoDproblem.FindDischargeLinkRtcl
            
            
            % Original definition
            % res = obj.FindChargeLinkRtcl(t,obj.spec.n_charge_step,obj.spec.n_charger) + obj.spec.n_charger*(obj.n_passenger_flow_in_optimization + 1)*(c-1) + obj.spec.n_charger*(k-1) + i;
            % We cache the calls to obj.FindChargeLinkRtcl for better performance
            res = obj.offset_FindDischargeLinkHelpertckl_t(t) + obj.spec.n_charger*(obj.n_passenger_flow_in_optimization + 1)*(c-1) + obj.spec.n_charger*(k-1) + i;
        end    
    end
    
    properties (Access = protected)
        n_passenger_flow_in_optimization(1,1) double  % Number of passenger flows that are present in the optimization. Is equal to spec.n_passenger_flow in the normal case and zero in the real-time formulation  
        optimization_variables(1,1) % Struct with optimization variables 
        road_residual_capacity_matrix(:,:,:) double % road_residual_capacity_matrix(i,j,t) is the residual capacity of the i-j link (in vehicles per unit time) at time t, after subtracting the flow from pre-routed vehicles      
    end
    
    methods (Access = private)
        charger_power_demand_w = ComputeChargerPowerDemand(obj,varargin)
        charger_power_demand = ComputeChargerPowerDemandNormalized(obj,factor,varargin)
        A_charger_power_w = ComputeChargerPowerMatrix(obj)        
        electricity_cost_usd = ComputeElectricityCost(obj,varargin)
        optimization_variables = DefineDecisionVariables(obj)        
        objective = GetObjective(obj)
        SetOffsetsForIndexingFunctions(obj) 
    end
       
    properties (Access = private)        
        % Cached offsets for indexing functions. See SetOffsetsForIndexingFunctions
        
        offset_FindChargeLinkHelpertckij_t(:,1) double % offset_FindChargeLinkHelpertckij_t(t ) is the offset for FindChargeLinkHelpertckij at t
        offset_FindDischargeLinkHelpertckl_t(:,1) double % offset_FindDischargeLinkHelpertckl_t(t) is the offset for FindDischargeLinkHelpertckl at t
    
        offset_FindPaxSourceChargecks(1,1) double % Offset for FindPaxSourceChargecks
        offset_FindPaxSinkChargetck(1,1) double % Offset for FindPaxSinkChargetck
        offset_FindEndRebLocationci(1,1) double % Offset for FindEndRebLocationci
    end
end



















