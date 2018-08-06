function ValidateRoadGraph(obj)
% ValidateRoadGraph asserts that the neighbor lists in
% obj.RoadGraph point to nodes that are integer and in [0,N].

for i = 1:obj.N
    neighbor_vec = obj.RoadGraph{i};
    assert(all(obj.IsValidRoadNode(neighbor_vec)),'Not all neighbors of node %i are valid road nodes',i)
end
end