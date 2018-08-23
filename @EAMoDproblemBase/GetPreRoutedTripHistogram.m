function pre_routed_trip_histogram = GetPreRoutedTripHistogram(obj)

pre_routed_trip_histogram = zeros(1,obj.spec.Thor);

for i_sink = 1:obj.spec.NumSinks
    sink = obj.spec.Sinks(i_sink);
    
    source_array = obj.spec.Sources{i_sink};
    
    for i_source = 1:numel(source_array)
        source = source_array(i_source);
        start_time = obj.spec.StartTimes{i_sink}(i_source);
        flow = obj.spec.Flows{i_sink}(i_source);
        
        route_time = obj.RouteTime(source,sink);
        
        end_time = start_time + route_time - 1;
        
        % All trips should have ended by Thor.
        assert(end_time < obj.spec.Thor);
        
        time_range = start_time:end_time;
        
        pre_routed_trip_histogram(time_range) = pre_routed_trip_histogram(time_range) + flow;
    end
end
end