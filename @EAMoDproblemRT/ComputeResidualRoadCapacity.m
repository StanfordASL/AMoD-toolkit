function road_residual_capacity_matrix = ComputeResidualRoadCapacity(obj)
% ComputeResidualRoadCapacity Computes the residual road capacity after accounting for the pre-routed passenger-carrying vehicles used in the real-time formulation

[obj.route_travel_time_matrix,obj.route_charge_to_traverse_matrix,obj.route_travel_distance_matrix_m,obj.route_path_cell] = obj.BuildRoutes();

road_utilization_matrix = zeros(obj.spec.n_road_node,obj.spec.n_road_node,obj.spec.n_time_step);

for i_sink = 1:obj.spec.n_passenger_sink
    sink = obj.spec.passenger_sink_list(i_sink);
    
    source_array = obj.spec.passenger_source_list_cell{i_sink};
    
    for i_source = 1:numel(source_array)
        source = source_array(i_source);
        flow = obj.spec.passenger_flow_list_cell{i_sink}(i_source);
        
        start_time = obj.spec.passenger_start_time_list_cell{i_sink}(i_source);
        current_time = start_time;
        
        route = obj.route_path_cell{source,sink};
        
        for i_route = 1:(numel(route) - 1)
            edge_source = route(i_route);
            edge_sink = route(i_route+1);
            
            edge_time = obj.spec.road_travel_time_matrix(edge_source,edge_sink);
            assert(edge_time > 0,'edge_time should be positive');
            
            final_edge_time = min(current_time + edge_time - 1,obj.spec.n_time_step);
                        
            road_utilization_matrix(edge_source,edge_sink,current_time:final_edge_time) = road_utilization_matrix(edge_source,edge_sink,current_time:final_edge_time) + flow;
            
            current_time = current_time + edge_time;
        end
        
        assert(current_time == start_time + obj.route_travel_time_matrix(source,sink),'current_time should be consistent with route_travel_time_matrix')
    end
end

ExpandedRoadCap = repmat(full(obj.spec.road_capacity_matrix),1,1,obj.spec.n_time_step);

road_residual_capacity_pre_matrix = ExpandedRoadCap - road_utilization_matrix;

% If any link is congested (negative road_residual_capacity_pre_matrix), set
% residual capacity to zero so that there is no rebalancing there
road_residual_capacity_matrix = max(road_residual_capacity_pre_matrix,0);

end