function ValidateRoadGraph(RoadGraph)
% ValidateRoadGraph asserts that the neighbor lists in
% RoadGraph point to nodes that are integer and in [0,N].

N = numel(obj.RoadGraph);

for i = 1:numel(obj.RoadGraph)
    neighbor_vec = RoadGraph{i};
    assert(all(obj.IsValidRoadNode(neighbor_vec)),'Not all neighbors of node %i are valid road nodes',i)
end
end