classdef EAMoDproblemBase < handle
    methods
        function obj = EAMoDproblemBase(spec)
            validateattributes(spec,{EAMoDspec},{'scalar'},mfilename,'spec',1);
             
            spec.ValidateSpec();
            obj.spec = spec;                        
                  
            [obj.RouteTime,obj.RouteCharge,obj.RouteDistance,obj.Routes] = obj.BuildRoutes();            
        end
        
        [f_cost,f_cost_pax,f_cost_reb,f_cost_relax] = CreateCostVector(obj);
        
        [A_eq,B_eq,A_in,B_in] = CreateConstraintMatrices(obj)
        
        [Aeq, Beq] = CreateEqualityConstraintMatrices_ConservationPax(obj);
        
        [Aineq, Bineq] = CreateInequalityConstraintMatrices(obj);
        
        [RouteTime,RouteCharge,RouteDistance,Routes] = BuildRoutes(obj);
        
        % State vector indexing functions
        
        function res = FindRoadLinkHelpertckij(obj,t,c,k,i,j)
            res = (t-1)*(obj.spec.E*(obj.num_passenger_flows + 1)*(obj.spec.C) + 2*(obj.num_passenger_flows+1)*obj.spec.NumChargers*(obj.spec.C)) + (c-1)*obj.spec.E*(obj.num_passenger_flows+1) + (k-1)*obj.spec.E + (obj.spec.cumRoadNeighbors(i) + obj.spec.RoadNeighborCounter(i,j));
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
                res = obj.FindRoadLinkPtckij(t,c,obj.spec.M + 1,i,j);
            end
        end
        
        function res = FindChargeLinkHelpertckij(obj,t,c,k,i)
            res = obj.FindRoadLinkRtcij(t,obj.spec.C,obj.spec.N,obj.spec.RoadGraph{end}(end)) + obj.spec.NumChargers*(obj.num_passenger_flows + 1)*(c-1) + obj.spec.NumChargers*(k-1) + i;  %Here we index the charger directly (as opposed to the node hosting the charger)
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
                res = obj.FindChargeLinkPtckl(t,c,obj.spec.M+1,i);
            end
        end
        
        function res = FindDischargeLinkHelpertckl(obj,t,c,k,i)
            res = obj.FindChargeLinkRtcl(t,obj.spec.C,obj.spec.NumChargers) + obj.spec.NumChargers*(obj.num_passenger_flows + 1)*(c-1) + obj.spec.NumChargers*(k-1) + i;
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
                res = obj.FindDischargeLinkPtckl(t,c,obj.spec.M + 1,i);
            end
        end
        
        function res = FindPaxSourceChargecks(obj,c,k,s)
            res = obj.FindDischargeLinkRtcl(obj.spec.Thor,obj.spec.C,obj.spec.NumChargers) + obj.spec.TotNumSources*(c-1) + obj.spec.CumNumSourcesPerSink(k) + s;
        end
        
        function res = FindPaxSinkChargetck(obj,t,c,k)
            res = obj.FindPaxSourceChargecks(obj.spec.C,obj.spec.M,obj.spec.NumSourcesPerSink(end)) + obj.spec.C*obj.spec.M*(t-1) + obj.spec.M*(c-1) + k;
        end
                
        function res = FindEndRebLocationci(obj,c,i)
            res = obj.FindPaxSinkChargetck(obj.spec.Thor,obj.spec.C,obj.spec.M) + obj.spec.N*(c-1) + i;
        end
        
        % Constraint Indexers
        function res = FindEqPaxConservationtcki(obj,t,c,k,i)
            res = obj.spec.N*obj.spec.M*obj.spec.C*(t - 1) + obj.spec.N*obj.spec.M*(c - 1) + obj.spec.N*(k - 1) + i;
        end
        
        % This is untested
        function [t,c,k,i] = ReverseFindEqPaxConservationtcki(obj,in)
            [t,c,k,i] = obj.DecomposeNumber(in,obj.spec.C, obj.spec.M, obj.spec.N);
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
            res = obj.spec.N*obj.spec.C*(t - 1) + obj.spec.N*(c - 1) + i;
        end
        
        function res = FindEqSourceConservationks(obj,k,s)
            res = obj.spec.CumNumSourcesPerSink(k) + s;
        end
        
        function res = FindEqSinkConservationk(obj,k)
            res = k;
        end
        
        function res = FindInRoadCongestiontij(obj,t,i,j)
            res = obj.spec.E*(t - 1) + obj.spec.cumRoadNeighbors(i) + obj.spec.RoadNeighborCounter(i,j);
        end
        
        function res = FindInChargerCongestiontl(obj,t,l)
            res = obj.spec.NumChargers*(t-1) + l;
        end
        
        % The indexing for FindSourceRelaxks is different from
        % TVPowerBalancedFlowFinder_sinkbundle because we do not include
        % the power network and we do not include the congestion relaxation
        function res = FindSourceRelaxks(obj,k,s)
            if obj.spec.sourcerelaxflag
                res = obj.FindEndRebLocationci(obj.spec.C,obj.spec.N) +  obj.spec.CumNumSourcesPerSink(k) + s;
            else
                res = nan;
            end
        end
        
        
        function res = get.StateSize(obj)
            if obj.spec.sourcerelaxflag
                res = obj.FindSourceRelaxks(obj.spec.NumSinks,obj.spec.NumSourcesPerSink(obj.spec.NumSinks));
            else
                res = obj.FindEndRebLocationci(obj.spec.C,obj.spec.N);
            end            
        end
        
        function res = get.num_passenger_flows(obj)
            if obj.use_real_time_formulation
                res = 0;
            else
                res = obj.spec.M; 
            end
        end
    end
    
    properties (Dependent)
        StateSize       
        num_passenger_flows
    end
    
    
    
    properties
        spec(1,1) EAMoDspec
        
        % Stuff for real time algorithm
        use_real_time_formulation(1,1) logical = false;
        RouteTime
        RouteCharge
        RouteDistance
        Routes        
    end
    
    
    
end