function obj = UpdatePropertiesDependentOnRoadGraph(obj)
% UpdatePropertiesDependentOnRoadGraph updates properties that depend on road_adjacency_list

obj.road_reverse_adjacency_list = cell(size(obj.road_adjacency_list));
for i=1:obj.n_road_node
    for j=obj.road_adjacency_list{i}
        obj.road_reverse_adjacency_list{j} = [obj.road_reverse_adjacency_list{j} i];
    end
end
for i=1:numel(obj.road_reverse_adjacency_list)
    obj.road_reverse_adjacency_list{i} = sort(unique(obj.road_reverse_adjacency_list{i}));
end

obj.road_adjacency_matrix = false(obj.n_road_node,obj.n_road_node);
for i = 1:obj.n_road_node
    for neighbor_c = obj.road_adjacency_list{i}
        obj.road_adjacency_matrix(i,neighbor_c) = 1;
    end
end

obj.n_road_edge = 0;
obj.road_node_outdegree = zeros(obj.n_road_node,1);
for i = 1:obj.n_road_node
    obj.road_node_outdegree(i) = length(obj.road_adjacency_list{i});
    obj.n_road_edge = obj.n_road_edge + length(obj.road_adjacency_list{i});
end

obj.edge_number_matrix = nan(obj.n_road_node,obj.n_road_node);

edge_counter = 0;
for i = 1:obj.n_road_node
    for j = obj.road_adjacency_list{i}(:).'
        edge_counter = 1 + edge_counter;
        obj.edge_number_matrix(i,j) = edge_counter;
    end    
end
end