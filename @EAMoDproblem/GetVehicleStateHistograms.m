function [ChargingVehicleHist,DischargingVehicleHist,PaxVehicleHist,...
    RebVehicleHist,IdleVehicleHist,AllVehicleHist] ...
    = GetVehicleStateHistograms(obj,varargin)
% GetVehicleStateHistograms Returns the distribution of vehicles across states as a function of time
%   [ChargingVehicleHist,DischargingVehicleHist,PaxVehicleHist,RebVehicleHist,IdleVehicleHist,AllVehicleHist] = GetVehicleStateHistograms(obj) uses the decision_vector in obj.decision_variables
%   [...] = GetVehicleStateHistograms(obj,decision_vector_val) uses decision_vector_val
%   ChargingVehicleHist contains the number of vehicles charging (including rebalancing and passenger-carrying),
%   DischargingVehicleHist contains the number of vehicles discharging (including rebalancing and passenger-carrying),
%   PaxVehicleHist contains the number of passenger-carrying moving vehicles
%   RebVehicleHist contains the number of rebalancing vehicles
%   IdleVehicleHist contains the number of idle rebalancing vehicles
%   AllVehicleHist contains the sum of all previous outputs

switch numel(varargin)
    case 0
        decision_vector_val = obj.EvaluateDecisionVector();
    case 1
        decision_vector_val = varargin{1};
    otherwise
        error('Too many arguments.')
end


% Charging vehicles
ChargingVehicleHist = zeros(1,obj.spec.Thor);
for tt = 1:obj.spec.Thor
    for c = 1:obj.spec.C
        for l = 1:obj.spec.NumChargers
            for deltat = 0:obj.spec.ChargerTime(l) - 1
                if tt - deltat > 0
                    for k = 1:obj.num_passenger_flows
                        ChargingVehicleHist(tt) = ChargingVehicleHist(tt) + decision_vector_val(obj.FindChargeLinkPtckl(tt-deltat,c,k,l));
                    end
                    ChargingVehicleHist(tt) = ChargingVehicleHist(tt) + decision_vector_val(obj.FindChargeLinkRtcl(tt-deltat,c,l));
                end
            end
        end
    end
end

% Discharging vehicles
DischargingVehicleHist = zeros(1,obj.spec.Thor);
for tt = 1:obj.spec.Thor
    for c = 1:obj.spec.C
        for l = 1:obj.spec.NumChargers
            for deltat = 0:obj.spec.ChargerTime(l) - 1
                if tt - deltat > 0
                    for k = 1:obj.num_passenger_flows
                        DischargingVehicleHist(tt) = DischargingVehicleHist(tt) + decision_vector_val(obj.FindDischargeLinkPtckl(tt-deltat,c,k,l));
                    end
                    DischargingVehicleHist(tt) = DischargingVehicleHist(tt) + decision_vector_val(obj.FindDischargeLinkRtcl(tt-deltat,c,l));
                end
            end
        end
    end
end

% Moving vehicles
PaxVehicleHist = zeros(1,obj.spec.Thor);
RebVehicleHist = zeros(1,obj.spec.Thor);
IdleVehicleHist = zeros(1,obj.spec.Thor);

for tt = 1:obj.spec.Thor
    for c = 1:obj.spec.C
        for i = 1:obj.spec.N
            if ~isempty(obj.spec.RoadGraph{i})
                for j = obj.spec.RoadGraph{i}
                    for deltat = 0:obj.spec.TravelTimes(i,j)-1
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
RebVehicleHist(obj.spec.Thor) = 0;

% IdleVehicleHist takes the final distribution of vehicles at Thor
n_end_vehicles = obj.ComputeNumberOfVehiclesAtEnd(varargin{:});
assert(IdleVehicleHist(end) == 0);
IdleVehicleHist(end) = n_end_vehicles;

AllVehicleHist = ChargingVehicleHist + DischargingVehicleHist + PaxVehicleHist + RebVehicleHist + IdleVehicleHist;
end







