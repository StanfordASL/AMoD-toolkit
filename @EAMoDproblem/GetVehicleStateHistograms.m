function [ChargingVehicleHist,DischargingVehicleHist,PaxVehicleHist,...
    RebVehicleHist,IdleVehicleHist,AllVehicleHist] ...
    = GetVehicleStateHistograms(obj,varargin)

switch numel(varargin)
    case 0
        decision_vector_val = obj.EvaluateDecisionVector();
    case 1
        decision_vector_val = varargin{1};
    otherwise
        error('Too many arguments.')    
end


% Charging vehicles
ChargingVehicleHist = zeros(1,obj.Thor);
for tt = 1:obj.Thor
    for c = 1:obj.C
        for l = 1:obj.NumChargers
            for k = 1:obj.num_passenger_flows
                ChargingVehicleHist(tt) = ChargingVehicleHist(tt) + decision_vector_val(obj.FindChargeLinkPtckl(tt,c,k,l));
            end
            ChargingVehicleHist(tt) = ChargingVehicleHist(tt) + decision_vector_val(obj.FindChargeLinkRtcl(tt,c,l));
        end
    end
end

% Discharging vehicles
DischargingVehicleHist = zeros(1,obj.Thor);
for tt = 1:obj.Thor
    for c = 1:obj.C
        for l = 1:obj.NumChargers
            for k = 1:obj.num_passenger_flows
                DischargingVehicleHist(tt) = DischargingVehicleHist(tt) + decision_vector_val(obj.FindDischargeLinkPtckl(tt,c,k,l));
            end
            DischargingVehicleHist(tt) = DischargingVehicleHist(tt) + decision_vector_val(obj.FindDischargeLinkRtcl(tt,c,l));
        end
    end
end

% Moving vehicles
PaxVehicleHist = zeros(1,obj.Thor);
RebVehicleHist = zeros(1,obj.Thor);
IdleVehicleHist = zeros(1,obj.Thor);

for tt = 1:obj.Thor
    for c = 1:obj.C
        for i = 1:obj.N
            if ~isempty(obj.RoadGraph{i})
                for j = obj.RoadGraph{i}
                    for deltat = 0:obj.TravelTimes(i,j)-1
                        if tt - deltat > 0
                            for k = 1:obj.num_passenger_flows
                                PaxVehicleHist(tt) = PaxVehicleHist(tt) + decision_vector_val(obj.FindRoadLinkPtckij(tt-deltat,c,k,i,j));
                            end
                            if i == j
                                % Idle vehicles are those that are
                                % rebalancing in the same node
                                IdleVehicleHist(tt) = IdleVehicleHist(tt) + decision_vector_val(obj.FindRoadLinkRtcij(tt-deltat,c,i,j));
                            else
                                RebVehicleHist(tt) = RebVehicleHist(tt) + decision_vector_val(obj.FindRoadLinkRtcij(tt-deltat,c,i,j));
                            end
                        end
                    end
                end
            end
        end
    end
end

% Account for PaxVehicle in real time formulation
if obj.use_real_time_formulation
    PaxVehicleHist = obj.GetPreRoutedTripHistogram();
end



% Note RebVehicleHist at Thor is meaningless. Since it is unconstrained and
% there are self-loops with distance zero it can take any value.
% Thus, we set it to zero.
RebVehicleHist(obj.Thor) = 0;

% IdleVehicleHist takes the final distribution of vehicles at Thor
n_end_vehicles = obj.ComputeNumberOfVehiclesAtEnd(varargin{:});
assert(IdleVehicleHist(end) == 0);
IdleVehicleHist(end) = n_end_vehicles;

AllVehicleHist = ChargingVehicleHist + DischargingVehicleHist + PaxVehicleHist + RebVehicleHist + IdleVehicleHist;
end








