function RemoveTripsEndingTooLate(obj)
% RemoveTripsEndingTooLate Removes trips ending after n_time_step which cannot be serviced

% Instantiate EAMoDproblemRT to compute routes
eamod_problem_rt = EAMoDproblemRT(obj);

end_time_list_cell = cell(obj.n_passenger_sink,1);
feasible_trip_list_cell = cell(obj.n_passenger_sink,1);

for i_sink = 1:obj.n_passenger_sink
    sink = obj.passenger_sink_list(i_sink);
    
    source_array = obj.passenger_source_list_cell{i_sink};
    
    n_source = numel(source_array);
    end_time_list = zeros(n_source,1);
    
    feasible_trip_list = false(n_source,1);
    
    for i_source = 1:n_source
        source = source_array(i_source);
        start_time = obj.passenger_start_time_list_cell{i_sink}(i_source);
        
        route_time = eamod_problem_rt.route_travel_time_matrix(source,sink);
        
        end_time = start_time + route_time - 1;
        
        end_time_list(i_source) = end_time;
        
        % All trips should have ended by n_time_step.
        feasible_trip_list(i_source) = end_time < obj.n_time_step;     
    end
    
    end_time_list_cell{i_sink} = end_time_list;
    feasible_trip_list_cell{i_sink} = feasible_trip_list;
end

passenger_sink_list_new = zeros(obj.n_passenger_sink,1);

passenger_source_list_cell_new = cell(obj.n_passenger_sink,1);
passenger_flow_list_cell_new = cell(obj.n_passenger_sink,1);
passenger_start_time_list_cell_new = cell(obj.n_passenger_sink,1);


i_sink = 0;
i_sink_new = 0;
while i_sink < obj.n_passenger_sink
    
    i_sink = i_sink + 1;
    
    feasible_trip_list = feasible_trip_list_cell{i_sink};
    
    if any(feasible_trip_list)
        i_sink_new = i_sink_new + 1;
        
        passenger_sink_list_new(i_sink_new) = obj.passenger_sink_list(i_sink);
        
        passenger_source_list_cell_new{i_sink_new} = obj.passenger_source_list_cell{i_sink}(feasible_trip_list);
        passenger_flow_list_cell_new{i_sink_new} = obj.passenger_flow_list_cell{i_sink}(feasible_trip_list);
        passenger_start_time_list_cell_new{i_sink_new} = obj.passenger_start_time_list_cell{i_sink}(feasible_trip_list);             
    end   
end

passenger_sink_list_new = passenger_sink_list_new(1:i_sink_new);
passenger_source_list_cell_new = passenger_source_list_cell_new(1:i_sink_new);
passenger_flow_list_cell_new = passenger_flow_list_cell_new(1:i_sink_new);
passenger_start_time_list_cell_new = passenger_start_time_list_cell_new(1:i_sink_new);

obj.passenger_sink_list = passenger_sink_list_new;
obj.passenger_source_list_cell = passenger_source_list_cell_new;
obj.passenger_flow_list_cell = passenger_flow_list_cell_new;
obj.passenger_start_time_list_cell = passenger_start_time_list_cell_new;
end