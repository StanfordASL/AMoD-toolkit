classdef EAMoDproblem < AbstractEAMoDproblem 
% EAMoDproblem Extends AbstractEAMoDproblem to implement the electric AMoD problem
        
    methods
        function obj = EAMoDproblem(spec)
            % EAMoDproblem Constructs an EAMoDproblem object to represent the electric AMoD problem
            %   obj = EAMoDproblem(spec) where spec is an instance of
            %   EAMoDspec specifying the problem.
            %
            %   See also EAMoDspec

            n_passenger_flow_in_optimization = spec.n_passenger_flow;
            obj@AbstractEAMoDproblem(spec,n_passenger_flow_in_optimization);           
        end
        
        % Equality constraints for standard formulation
        [Aeq_PaxConservation, Beq_PaxConservation] = CreateEqualityConstraintMatrices_PaxConservation(obj)
        
        
    end 
    
    methods        
        % State vector indexing functions
        
        function res = FindRoadLinkPtckij(obj,t,c,k,i,j)
            % FindRoadLinkPtckij Indexer for customer flow in road edges in the extended network
            
            res = obj.FindRoadLinkHelpertckij(t,c,k,i,j);
        end
        
        function res = FindChargeLinkPtckl(obj,t,c,k,i)
            % FindChargeLinkPtckl Indexer for customer flow in charging edges in the extended network
            
            res = obj.FindChargeLinkHelpertckij(t,c,k,i);
        end
        
        function res = FindDischargeLinkPtckl(obj,t,c,k,i)
            % FindDischargeLinkPtckl Indexer for customer flow in discharging edges in the extended network
            
            res = obj.FindDischargeLinkHelpertckl(t,c,k,i);
        end
        
        % Constraint Indexers
        
        function res = FindEqPaxConservationtcki(obj,t,c,k,i)
            % FindEqPaxConservationtcki Indexer for customer flow conservation constraints
            
            res = obj.spec.n_road_node*obj.spec.n_passenger_flow*obj.spec.n_charge_step*(t - 1) + obj.spec.n_road_node*obj.spec.n_passenger_flow*(c - 1) + obj.spec.n_road_node*(k - 1) + i;
        end
    end
    
    methods (Access = protected)        
        % Abstract methods in AbstractEAMoDproblem
        road_residual_capacity_matrix = ComputeResidualRoadCapacity(obj)
        constraint_array = GetConstraintArray(obj)
    end   

end