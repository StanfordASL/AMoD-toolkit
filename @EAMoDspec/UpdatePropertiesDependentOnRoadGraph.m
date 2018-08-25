function obj = UpdatePropertiesDependentOnRoadGraph(obj)
% UpdatePropertiesDependentOnRoadGraph updates properties that depend on RoadGraph

obj.ReverseRoadGraph = cell(size(obj.RoadGraph));
for i=1:obj.N
    for j=obj.RoadGraph{i}
        obj.ReverseRoadGraph{j} = [obj.ReverseRoadGraph{j} i];
    end
end
for i=1:numel(obj.ReverseRoadGraph)
    obj.ReverseRoadGraph{i} = sort(unique(obj.ReverseRoadGraph{i}));
end

obj.adjacency_matrix = false(obj.N,obj.N);
for i = 1:obj.N
    for neighbor_c = obj.RoadGraph{i}
        obj.adjacency_matrix(i,neighbor_c) = 1;
    end
end

obj.E = 0;
obj.NumRoadEdges = zeros(obj.N,1);
for i = 1:obj.N
    obj.NumRoadEdges(i) = length(obj.RoadGraph{i});
    obj.E = obj.E + length(obj.RoadGraph{i});
end

obj.edge_number_matrix = nan(obj.N,obj.N);

edge_counter = 0;
for i = 1:obj.N
    for j = obj.RoadGraph{i}(:).'
        edge_counter = 1 + edge_counter;
        obj.edge_number_matrix(i,j) = edge_counter;
    end    
end
end