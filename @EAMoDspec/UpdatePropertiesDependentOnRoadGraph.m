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

obj.cumRoadNeighbors = cumsum(obj.NumRoadEdges);
obj.cumRoadNeighbors=[0;obj.cumRoadNeighbors(1:end-1)];

obj.RoadNeighborCounter = sparse([],[],[],obj.N,obj.N,obj.E);
TempNeighVec = zeros(obj.N,1);
for i = 1:obj.N
    for j = obj.RoadGraph{i}
        TempNeighVec(j) = 1;
    end
    NeighCounterLine = cumsum(TempNeighVec);
    for j = obj.RoadGraph{i}
        obj.RoadNeighborCounter(i,j) = NeighCounterLine(j);
    end
    TempNeighVec = zeros(obj.N,1);
end
end