function [ChargingVehicleHist,DischargingVehicleHist,PaxVehicleHist,...
    RebVehicleHist,IdleVehicleHist,AllVehicleHist] ...
    = GetVehicleStateHistograms(obj,varargin)
% GetVehicleStateHistograms Returns the distribution of vehicles across states as a function of time
%   [ChargingVehicleHist,DischargingVehicleHist,PaxVehicleHist,RebVehicleHist,IdleVehicleHist,AllVehicleHist] = GetVehicleStateHistograms(obj) uses the state_vector in obj.optimization_variables
%   [...] = GetVehicleStateHistograms(obj,state_vector_val) uses state_vector_val
%   ChargingVehicleHist contains the number of vehicles charging (including rebalancing and passenger-carrying),
%   DischargingVehicleHist contains the number of vehicles discharging (including rebalancing and passenger-carrying),
%   PaxVehicleHist contains the number of passenger-carrying moving vehicles
%   RebVehicleHist contains the number of rebalancing vehicles
%   IdleVehicleHist contains the number of idle rebalancing vehicles
%   AllVehicleHist contains the sum of all previous outputs

switch numel(varargin)
    case 0
        state_vector_val = obj.EvaluateStateVector();
    case 1
        state_vector_val = varargin{1};
    otherwise
        error('Too many arguments.')
end

% Charging vehicles
ChargingVehicleHist = zeros(1,obj.spec.n_time_step);
for tt = 1:obj.spec.n_time_step
    for c = 1:obj.spec.n_charge_step
        for l = 1:obj.spec.n_charger
            for deltat = 0:obj.spec.charger_time(l) - 1
                if tt - deltat > 0
                    for k = 1:obj.n_passenger_flow_in_optimization
                        ChargingVehicleHist(tt) = ChargingVehicleHist(tt) + state_vector_val(obj.FindChargeLinkPtckl(tt-deltat,c,k,l));
                    end
                    ChargingVehicleHist(tt) = ChargingVehicleHist(tt) + state_vector_val(obj.FindChargeLinkRtcl(tt-deltat,c,l));
                end
            end
        end
    end
end

% Discharging vehicles
DischargingVehicleHist = zeros(1,obj.spec.n_time_step);
for tt = 1:obj.spec.n_time_step
    for c = 1:obj.spec.n_charge_step
        for l = 1:obj.spec.n_charger
            for deltat = 0:obj.spec.charger_time(l) - 1
                if tt - deltat > 0
                    for k = 1:obj.n_passenger_flow_in_optimization
                        DischargingVehicleHist(tt) = DischargingVehicleHist(tt) + state_vector_val(obj.FindDischargeLinkPtckl(tt-deltat,c,k,l));
                    end
                    DischargingVehicleHist(tt) = DischargingVehicleHist(tt) + state_vector_val(obj.FindDischargeLinkRtcl(tt-deltat,c,l));
                end
            end
        end
    end
end

% Moving vehicles
PaxVehicleHist = zeros(1,obj.spec.n_time_step);
RebVehicleHist = zeros(1,obj.spec.n_time_step);
IdleVehicleHist = zeros(1,obj.spec.n_time_step);

for tt = 1:obj.spec.n_time_step
    for c = 1:obj.spec.n_charge_step
        for i = 1:obj.spec.n_road_node
            if ~isempty(obj.spec.road_adjacency_list{i})
                for j = obj.spec.road_adjacency_list{i}
                    for deltat = 0:obj.spec.road_travel_time_matrix(i,j)-1
                        if tt - deltat > 0
                            for k = 1:obj.n_passenger_flow_in_optimization
                                PaxVehicleHist(tt) = PaxVehicleHist(tt) + state_vector_val(obj.FindRoadLinkPtckij(tt-deltat,c,k,i,j));
                            end
                            if i == j
                                % Idle vehicles are those that are
                                % rebalancing in the same node
                                IdleVehicleHist(tt) = IdleVehicleHist(tt) + state_vector_val(obj.FindRoadLinkRtcij(tt-deltat,c,i,j));
                            else
                                RebVehicleHist(tt) = RebVehicleHist(tt) + state_vector_val(obj.FindRoadLinkRtcij(tt-deltat,c,i,j));
                            end
                        end
                    end
                end
            end
        end
    end
end

% Note RebVehicleHist at n_time_step is meaningless. Since it is unconstrained and
% there are self-loops with distance zero it can take any value.
% Thus, we set it to zero.
RebVehicleHist(obj.spec.n_time_step) = 0;

% IdleVehicleHist takes the final distribution of vehicles at n_time_step
n_end_vehicles = obj.ComputeNumberOfVehiclesAtEnd(varargin{:});
assert(IdleVehicleHist(end) == 0);
IdleVehicleHist(end) = n_end_vehicles;

AllVehicleHist = ChargingVehicleHist + DischargingVehicleHist + PaxVehicleHist + RebVehicleHist + IdleVehicleHist;
end








