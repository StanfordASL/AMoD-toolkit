function pre_routed_trip_histogram = GetPreRoutedTripHistogram(obj)
% GetPreRoutedTripHistogram Returns the number of pre-routed passenger-carrying vehicles as a function of time

pre_routed_trip_histogram = zeros(1,obj.spec.n_time_step);

for i_sink = 1:obj.spec.n_passenger_sink
    sink = obj.spec.passenger_sink_list(i_sink);
    
    source_array = obj.spec.passenger_source_list_cell{i_sink};
    
    for i_source = 1:numel(source_array)
        source = source_array(i_source);
        start_time = obj.spec.passenger_start_time_list_cell{i_sink}(i_source);
        flow = obj.spec.passenger_flow_list_cell{i_sink}(i_source);
        
        route_time = obj.RouteTime(source,sink);
        
        end_time = start_time + route_time - 1;
        
        % All trips should have ended by n_time_step.
        assert(end_time < obj.spec.n_time_step);
        
        time_range = start_time:end_time;
        
        pre_routed_trip_histogram(time_range) = pre_routed_trip_histogram(time_range) + flow;
    end
end
end