function obj = UpdatePropertiesDependentOnSources(obj)
% UpdatePropertiesDependentOnSources updates cached properties that depend on Sources

obj.NumSourcesPerSink = cellfun(@numel,obj.Sources);

obj.CumNumSourcesPerSink = cumsum(obj.NumSourcesPerSink);
obj.TotNumSources = obj.CumNumSourcesPerSink(end);
obj.CumNumSourcesPerSink=[0; obj.CumNumSourcesPerSink(1:end-1)];
end