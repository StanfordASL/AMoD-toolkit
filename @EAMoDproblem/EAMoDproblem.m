% This integrates EAMoDproblemBase with Yalmip
classdef EAMoDproblem < EAMoDproblemBase
    methods
        function obj = EAMoDproblem(spec)
            obj@EAMoDproblemBase(spec);
        end
        
        Initialize(obj)
        
        
        [total_cost_val, pax_cost_val, reb_cost_val,relax_cost_val] = EvaluateAMoDcost(obj);
        
        [objective_value,solver_time,diagnostics]  = Solve(obj);
        
        relax_range = GetRelaxRange(obj);
        state_range = GetStateRange(obj);
        
        A_charger_power_w = ComputeChargerPowerMatrix(obj);
        A_charger_power_w = ComputeChargerPowerMatrixNew(obj);
        
        charger_power_demand_w = GetChargerPowerDemand(obj)
        charger_power_demand = GetChargerPowerDemandNormalized(obj,factor);
        
        
        charger_power_demand_w_val = EvaluateChargerPowerDemand(obj,varargin)
        
        [DepTimeHist, ArrivalTimeHist] = GetTravelTimesHistograms(obj,varargin)
        
        [ChargingVehicleHist,DischargingVehicleHist,PaxVehicleHist,...
            RebVehicleHist,IdleVehicleHist,AllVehicleHist] ...
            = GetVehicleStateHistograms(obj,varargin)
        
        n_start_vehicles = ComputeNumberOfVehiclesAtStart(obj);
        n_end_vehicles = ComputeNumberOfVehiclesAtEnd(obj,varargin);
        
        
        E_charge_min_j = ComputeMinimalChargingEnergy(obj);
        
        function decision_vector_val = EvaluateDecisionVector(obj)
            decision_vector_val = value(obj.decision_variables.decision_vector);
        end
        
        
        
        final_vehicle_distribution = GetFinalVehicleDistribution(obj,varargin);
        
        
        
        [E_charge_min_j,P_charge_min_flat_w] = ComputeMinimalChargingRequirements(obj);
        E_charge_j = EvaluateChargerEnergyDemand(obj,varargin)
        
        stored_energy_j = ComputeStoredEnergyAtStart(obj);
        stored_energy_j = ComputeStoredEnergyAtEnd(obj);
        
        pax_cost_rt = ComputePaxCostRealTimeFormulation(obj);
        
        h = PlotVehicleState(obj,params_plot,title_text,varargin)
        
        pre_routed_trip_histogram = GetPreRoutedTripHistogram(obj);
        
        h = PlotTravelTimes(obj,params_plot,title_text,varargin)
    end
    
    methods %(Access = ?DAMoDproblem)
        constraint_array = DefineConstraints(obj);
        [total_cost, pax_cost,reb_cost, relax_cost] = ComputeAMoDcost(obj);
    end
    
    methods %(Access = protected)
        decision_variables = DefineDecisionVariables(obj);
    end
    
    properties % (Access = protected)
        decision_variables
        constraint_array
    end
    
    properties (SetAccess = protected, GetAccess = public)
        is_initialized(1,1) logical = false;
        state_range
        relax_range
    end
    
    
    
end