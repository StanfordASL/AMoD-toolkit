classdef EAMoDproblemRT < AbstractEAMoDproblem
    
    methods
        function obj = EAMoDproblemRT(spec)
            n_passenger_flow_in_optimization = 0;
            obj@AbstractEAMoDproblem(spec,n_passenger_flow_in_optimization);            
        end
        
        [lb_StateVector,ub_StateVector] = CreateStateVectorBounds(obj)
        
        [Aeq_CustomerChargeConservation, Beq_CustomerChargeConservation] = CreateEqualityConstraintMatrices_CustomerChargeConservation(obj)
        
        [ChargingVehicleHist,DischargingVehicleHist,PaxVehicleHist,RebVehicleHist,IdleVehicleHist,AllVehicleHist] = GetVehicleStateHistograms(obj,varargin)
       
    end
    
    properties (SetAccess = private, GetAccess = public)
        route_travel_time_matrix(:,:)  double {mustBeNonnegative,mustBeReal,mustBeInteger}  % route_travel_time_matrix(i,j) is the number of time-steps needed to go from i to j
        route_charge_to_traverse_matrix(:,:)  double {mustBeNonnegative,mustBeReal,mustBeInteger} % route_charge_to_traverse_matrix(i,j) is the number of charge units needed to go from i to j
        route_travel_distance_matrix_m(:,:)  double {mustBeNonnegative,mustBeReal} % route_travel_distance_matrix_m(i,j) is the distance in meters to go from i to j
        route_path_cell(:,:) cell % route_path_cell{i,j} is the route from i to j expressed as a vector of connected nodes that need to be traversed
    end
    
    
    
    methods (Access = private)
        [route_travel_time_matrix,route_charge_to_traverse_matrix,route_travel_distance_matrix_m,route_path_cell] = BuildRoutes(obj)
        pax_cost_rt = ComputePaxCostRealTimeFormulation(obj)   
        pre_routed_trip_histogram = GetPreRoutedTripHistogram(obj)
    end
    
    methods (Access = protected) 
        [amod_cost_usd, pax_cost_usd,reb_cost_usd, relax_cost_usd] = ComputeAMoDcost(obj,varargin)
        
        % Abstract methods in AbstractEAMoDproblem
        road_residual_capacity_matrix = ComputeResidualRoadCapacity(obj)
        constraint_array = GetConstraintArray(obj)
    end
end