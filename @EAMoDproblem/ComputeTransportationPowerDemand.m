function transportation_power_demand_w = ComputeTransportationPowerDemand(obj)

decision_vector_val = obj.EvaluateDecisionVector();
pax_vehicle_charge_demand = zeros(obj.Thor,1);
reb_vehicle_charge_demand = zeros(obj.Thor,1);

for tt = 1:obj.Thor
    for c = 1:obj.C
        for i = 1:obj.N
            if ~isempty(obj.RoadGraph{i})
                for j = obj.RoadGraph{i}
                    charge_demand = obj.ChargeToTraverse(i,j)/obj.TravelTimes(i,j);                   
                    for deltat = 0:obj.TravelTimes(i,j)-1 
                        if tt - deltat > 0
                            for k = 1:obj.num_passenger_flows
                                pax_vehicle_charge_demand(tt) = pax_vehicle_charge_demand(tt) + charge_demand*decision_vector_val(obj.FindRoadLinkPtckij(tt-deltat,c,k,i,j));
                            end
                            reb_vehicle_charge_demand(tt) = reb_vehicle_charge_demand(tt) + charge_demand*decision_vector_val(obj.FindRoadLinkRtcij(tt-deltat,c,i,j));
                        end
                    end                    
                end
            end
        end
    end
end

% % Account for PaxVehicle in real time formulation
% if obj.use_real_time_formulation
%     for i_sink = 1:obj.NumSinks
%         sink = obj.Sinks(i_sink);
% 
%         source_array = obj.Sources{i_sink};
% 
%         for i_source = 1:numel(source_array)
%            source = source_array(i_source);
%            start_time = obj.StartTimes{i_sink}(i_source);
%            flow = obj.Flows{i_sink}(i_source);
% 
%            % We account for all power demand in the first time step of the
%            % route
%            charge_demand_route = obj.RouteCharge(source,sink);
%          
%            pax_vehicle_charge_demand(start_time) = pax_vehicle_charge_demand(start_time) + charge_demand_route*flow;           
%         end    
%     end
% end

% Since we book charge at start, we need to book demand at end
if obj.use_real_time_formulation
    for k = 1:obj.M
        for tt = 1:obj.Thor
            for c = 1:obj.C
                for ssi = 1 :length(obj.Sources{k})
                    start_times = obj.StartTimes{k}(ssi);
                    start_charge = c + obj.RouteCharge(obj.Sources{k}(ssi),obj.Sinks(k));
                    if start_times == tt - obj.RouteTime(obj.Sources{k}(ssi),obj.Sinks(k))                             
                        if start_charge <= obj.C && start_charge > 0                
                            flow = decision_vector_val(obj.FindPaxSourceChargecks(start_charge,k,ssi));                                                        
                            pax_vehicle_charge_demand(tt) = pax_vehicle_charge_demand(tt) + obj.RouteCharge(obj.Sources{k}(ssi),obj.Sinks(k))*flow;
                        end
                    end
                end
            end
        end
    end
end


transportation_charge_demand_j = pax_vehicle_charge_demand*obj.charge_unit_j;
transportation_power_demand_w = transportation_charge_demand_j/obj.time_step_s;
end