function stored_energy_j = ComputeStoredEnergy(obj)

decision_vector_val = obj.EvaluateDecisionVector();

stored_energy = zeros(obj.Thor,1);

% Charging vehicles
for tt = 1:obj.Thor
    for c = 1:obj.C
        for l = 1:obj.NumChargers
            for k = 1:obj.num_passenger_flows
                stored_energy(tt) = stored_energy(tt) + c*decision_vector_val(obj.FindChargeLinkPtckl(tt,c,k,l));
            end
            stored_energy(tt) = stored_energy(tt) + c*decision_vector_val(obj.FindChargeLinkRtcl(tt,c,l));
        end
    end
end

% Discharging vehicles
for tt = 1:obj.Thor
    for c = 1:obj.C
        for l = 1:obj.NumChargers
            for k = 1:obj.num_passenger_flows
                stored_energy(tt) = stored_energy(tt) + c*decision_vector_val(obj.FindDischargeLinkPtckl(tt,c,k,l));
            end
            stored_energy(tt) = stored_energy(tt) + c*decision_vector_val(obj.FindDischargeLinkRtcl(tt,c,l));
        end
    end
end

% Moving vehicles
for tt = 1:obj.Thor
    for c = 1:obj.C
        for i = 1:obj.N
            if ~isempty(obj.RoadGraph{i})
                for j = obj.RoadGraph{i}
                    for deltat = 0:obj.TravelTimes(i,j)-1
                        if tt - deltat > 0
                            for k = 1:obj.num_passenger_flows
                                stored_energy(tt) = stored_energy(tt) + c*decision_vector_val(obj.FindRoadLinkPtckij(tt-deltat,c,k,i,j));
                            end
                            stored_energy(tt) = stored_energy(tt) + c*decision_vector_val(obj.FindRoadLinkRtcij(tt-deltat,c,i,j));
                        end
                    end
                end
            end
        end
    end
end

% % Book energy at end
% if obj.use_real_time_formulation
%     % This is based on
%     % CreateEqualityConstraintMatrices_CustomerChargeConservation. 
%     % When a customer boards a vehicle, we account for the energy that will
%     % be remain at the end of the trip given by
%     % obj.FindPaxSinkChargetck(t,c,k). This is for consistency with
%     % ComputeTransportationPowerDemand, where we account for the trip's energy
%     % demand at the begining.
%     for k = 1:obj.M
%         for tt = 1:obj.Thor
%             for c = 1:obj.C
%                 % This is only here for reference to FindPaxSinkChargetck.
%                 %Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindPaxSinkChargetck(t,c,k),-1];
%                 for ssi = 1 :length(obj.Sources{k})
%                     if obj.StartTimes{k}(ssi) == tt - obj.RouteTime(obj.Sources{k}(ssi),obj.Sinks(k))                             
%                         if c + obj.RouteCharge(obj.Sources{k}(ssi),obj.Sinks(k)) <= obj.C && c + obj.RouteCharge(obj.Sources{k}(ssi),obj.Sinks(k)) > 0                            
%                             stored_energy(tt) = stored_energy(tt) + c*decision_vector_val(obj.FindPaxSourceChargecks(c + obj.RouteCharge(obj.Sources{k}(ssi),obj.Sinks(k)),k,ssi));                            
%                         end
%                     end
%                 end
%             end
%         end
%     end
% end

% Book energy at start
if obj.use_real_time_formulation
    for k = 1:obj.M
        for tt = 1:obj.Thor
            for c = 1:obj.C
                % This is only here for reference to FindPaxSinkChargetck.
                %Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindPaxSinkChargetck(t,c,k),-1];
                for ssi = 1 :length(obj.Sources{k})
                    start_time = obj.StartTimes{k}(ssi);
                    route_time = obj.RouteTime(obj.Sources{k}(ssi),obj.Sinks(k));
                    end_time = start_time + route_time - 1;
                    start_charge = c + obj.RouteCharge(obj.Sources{k}(ssi),obj.Sinks(k));
                   
                    if start_time == tt - route_time                  
                        if start_charge <= obj.C && start_charge > 0                            
                            stored_energy(start_time:end_time) = stored_energy(start_time:end_time) + start_charge*decision_vector_val(obj.FindPaxSourceChargecks(start_charge,k,ssi));                            
                        end
                    end
                end
            end
        end
    end
end



stored_energy_j = stored_energy*obj.charge_unit_j;

% Energy at end is computed from the end distribution
stored_energy_end_j = obj.ComputeStoredEnergyAtEnd();

stored_energy_j(end) = stored_energy_end_j;
end