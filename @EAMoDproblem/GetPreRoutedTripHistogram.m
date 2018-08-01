function pre_routed_trip_histogram = GetPreRoutedTripHistogram(obj)

pre_routed_trip_histogram = zeros(1,obj.Thor);

for i_sink = 1:obj.NumSinks
    sink = obj.Sinks(i_sink);
    
    source_array = obj.Sources{i_sink};    
    
    for i_source = 1:numel(source_array)
        source = source_array(i_source);
        start_time = obj.StartTimes{i_sink}(i_source);
        flow = obj.Flows{i_sink}(i_source);
        
        route_time = obj.RouteTime(source,sink);
        
        end_time = start_time + route_time - 1;
        
        % All trips should have ended by Thor.
        assert(end_time < obj.Thor);
        
        time_range = start_time:end_time;
        
        pre_routed_trip_histogram(time_range) = pre_routed_trip_histogram(time_range) + flow;
    end
end
end