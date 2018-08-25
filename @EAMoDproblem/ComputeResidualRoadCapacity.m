function TVRoadCap = ComputeResidualRoadCapacity(obj)

road_utilization_matrix = zeros(obj.spec.N,obj.spec.N,obj.spec.Thor);

for i_sink = 1:obj.spec.NumSinks
    sink = obj.spec.Sinks(i_sink);
    
    source_array = obj.spec.Sources{i_sink};
    
    for i_source = 1:numel(source_array)
        source = source_array(i_source);
        flow = obj.spec.Flows{i_sink}(i_source);
        
        start_time = obj.spec.StartTimes{i_sink}(i_source);
        current_time = start_time;
        
        route = obj.Routes{source,sink};
        
        for i_route = 1:(numel(route) - 1)
            edge_source = route(i_route);
            edge_sink = route(i_route+1);
            
            edge_time = obj.spec.TravelTimes(edge_source,edge_sink);
            assert(edge_time > 0,'edge_time should be positive');
            
            final_edge_time = min(current_time + edge_time - 1,obj.spec.Thor);
                        
            road_utilization_matrix(edge_source,edge_sink,current_time:final_edge_time) = road_utilization_matrix(edge_source,edge_sink,current_time:final_edge_time) + flow;
            
            current_time = current_time + edge_time;
        end
        
        assert(current_time == start_time + obj.RouteTime(source,sink),'current_time should be consistent with RouteTime')
    end
end

ExpandedRoadCap = repmat(full(obj.spec.RoadCap),1,1,obj.spec.Thor);

TVRoadCap = ExpandedRoadCap - road_utilization_matrix;
end