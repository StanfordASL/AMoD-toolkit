function aaExperiment_EdgeCounter
data = load('EAMoDspecDemo.mat');
spec = data.spec;

edge_number_matrix = zeros(spec.N,spec.N);

for i = 1:spec.N
    for j = 1:spec.N
        edge_number_matrix(i,j) = spec.cumRoadNeighbors(i) + spec.RoadNeighborCounter(i,j);
    end
    
end

assert(all(edge_number_matrix(spec.adjacency_matrix) == spec.edge_number_matrix(spec.adjacency_matrix)));
end