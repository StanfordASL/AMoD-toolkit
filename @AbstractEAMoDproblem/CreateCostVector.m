function [f_cost,f_cost_pax,f_cost_reb,f_cost_relax] = CreateCostVector(obj)
% CreateCostVector Creates the cost vector for the linear program
%   [f_cost,f_cost_pax,f_cost_reb,f_cost_relax] = CreateCostVector(obj)
%   where f_cost is used for the total cost, f_cost_pax for the cost
%   associated with passenger-carrying vehicles, f_cost_reb for the cost
%   associated with rebalancing trips and f_cost_relax for the cost
%   associated with the relaxations.

f_cost_pax = zeros(obj.n_state_vector,1);
f_cost_reb = zeros(obj.n_state_vector,1);
f_cost_relax = zeros(obj.n_state_vector,1);

% Passengers' and rebalancers' travel time and battery depreciation
for t = 1:obj.spec.n_time_step
    for i = 1:obj.spec.n_road_node
        for j = obj.spec.road_adjacency_list{i}
            for c = 1:obj.spec.n_charge_step
                % Loop does not run for obj.n_passenger_flow_in_optimization = 0
                for k = 1:obj.n_passenger_flow_in_optimization
                    % Recall that road_travel_distance_matrix_m is in meter
                    f_cost_pax(obj.FindRoadLinkPtckij(t,c,k,i,j)) = obj.spec.value_of_time_usd_per_time_step*obj.spec.road_travel_time_matrix(i,j) + obj.spec.road_travel_distance_matrix_m(i,j)*obj.spec.vehicle_cost_usd_per_m + abs(obj.spec.road_charge_to_traverse_matrix(i,j))*obj.spec.battery_depreciation_usd_per_charge_step;
                end
                if j ~= i
                    % Recall that road_travel_distance_matrix_m is in meter
                    f_cost_reb(obj.FindRoadLinkRtcij(t,c,i,j)) = obj.spec.road_travel_distance_matrix_m(i,j)*obj.spec.vehicle_cost_usd_per_m + abs(obj.spec.road_charge_to_traverse_matrix(i,j))*obj.spec.battery_depreciation_usd_per_charge_step;
                end
            end
        end
    end
end

% Passengers' and rebalancers' charging time and cost  plus battery depreciation
for t = 1:obj.spec.n_time_step
    for i = 1:length(obj.spec.charger_list)
        for c = 1:obj.spec.n_charge_step
            % Loop does not run for obj.n_passenger_flow_in_optimization = 0
            for k = 1:obj.n_passenger_flow_in_optimization
                f_cost_pax(obj.FindChargeLinkPtckl(t,c,k,i)) = obj.spec.value_of_time_usd_per_time_step*obj.spec.charger_time(i) + abs(obj.spec.charger_speed(i))*obj.spec.battery_depreciation_usd_per_charge_step;
                f_cost_pax(obj.FindDischargeLinkPtckl(t,c,k,i)) = obj.spec.value_of_time_usd_per_time_step*obj.spec.charger_time(i) + abs(obj.spec.charger_speed(i))*obj.spec.battery_depreciation_usd_per_charge_step;
            end
            f_cost_reb(obj.FindChargeLinkRtcl(t,c,i))= abs(obj.spec.charger_speed(i))*obj.spec.battery_depreciation_usd_per_charge_step;
            f_cost_reb(obj.FindDischargeLinkRtcl(t,c,i))= abs(obj.spec.charger_speed(i))*obj.spec.battery_depreciation_usd_per_charge_step;
        end
    end
end

% Cost for source relaxation
if obj.source_relax_flag
    f_cost_relax(obj.FindSourceRelaxks(1,1):obj.FindSourceRelaxks(obj.spec.n_passenger_flow,length(obj.spec.passenger_source_list_cell{obj.spec.n_passenger_flow}))) = obj.source_relax_cost;
end

f_cost = f_cost_pax + f_cost_reb + f_cost_relax;
end