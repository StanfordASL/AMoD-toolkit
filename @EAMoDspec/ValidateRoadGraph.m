function ValidateRoadGraph(obj)
% ValidateRoadGraph asserts that the neighbor lists in obj.road_adjacency_list point to valid nodes.
% See also EAMoDspec.IsValidRoadNode

for i = 1:obj.n_road_node
    neighbor_vec = obj.road_adjacency_list{i};
    assert(all(obj.IsValidRoadNode(neighbor_vec)),'Not all neighbors of node %i are valid road nodes',i)
end
end