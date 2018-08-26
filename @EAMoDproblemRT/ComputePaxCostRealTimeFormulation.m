function pax_cost_rt = ComputePaxCostRealTimeFormulation(obj)
% ComputePaxCostRealTimeFormulation Computes the cost  associated with the pre-routed passenger-carrying vehicles in the real-time formulation
%   In the real-time formulation, passenger carrying vehicles follow pre-computed routes and are not part of the optimization.
%
%   See also EAMoDproblem.BuildRoutes

time_cost = 0;
distance_cost = 0;
battery_depreciation_cost = 0;

for i_sink = 1:obj.spec.n_passenger_sink
    sink = obj.spec.passenger_sink_list(i_sink);
    
    source_array = obj.spec.passenger_source_list_cell{i_sink};
    
    for i_source = 1:numel(source_array)
        source = source_array(i_source);
        flow = obj.spec.passenger_flow_list_cell{i_sink}(i_source);
        
        route_time = obj.route_travel_time_matrix(source,sink);
        time_cost = time_cost + flow*route_time*obj.spec.value_of_time_usd_per_time_step;
        
        route_distance = obj.route_travel_distance_matrix_m(source,sink);
        distance_cost = distance_cost + flow*route_distance*obj.spec.vehicle_cost_usd_per_m;
        
        route_charge = obj.route_charge_to_traverse_matrix(source,sink);
        battery_depreciation_cost = battery_depreciation_cost + flow*abs(route_charge)*obj.spec.battery_depreciation_usd_per_charge_step;
    end
end

pax_cost_rt = time_cost + distance_cost;
end