function obj = UpdatePropertiesDependentOnSources(obj)
% UpdatePropertiesDependentOnSources updates cached properties that depend on passenger_source_list_cell

obj.n_sources_to_sink = cellfun(@numel,obj.passenger_source_list_cell);

obj.n_sources_to_sink_cumsum = cumsum(obj.n_sources_to_sink);
obj.n_passenger_source = obj.n_sources_to_sink_cumsum(end);
obj.n_sources_to_sink_cumsum=[0; obj.n_sources_to_sink_cumsum(1:end-1)];
end