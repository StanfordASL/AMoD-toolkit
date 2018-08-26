function ValidateConsistencyWithAdjancencyMatrix(obj,n_passenger_flow,varargin)
% ValidateConsistencyWithAdjancencyMatrix asserts that a matrix n_passenger_flow has the same size as road_adjacency_matrix and that n_passenger_flow is zero when road_adjacency_matrix is zero.
%   ValidateConsistencyWithAdjancencyMatrix(obj,n_passenger_flow)
%   ValidateConsistencyWithAdjancencyMatrix(obj,n_passenger_flow,M_name) where
%   M_name is the name of n_passenger_flow to be included in error messages

n_varargin = numel(varargin);

switch n_varargin
    case 0
        M_name = 'n_passenger_flow';
    case 1
        M_name = varargin{1};
        
    otherwise
        error('Too many arguments')
end

assert(all(size(n_passenger_flow) == size(obj.road_adjacency_matrix)),'Size of %s must match that of road_adjacency_matrix.',M_name)
assert(all(n_passenger_flow(~obj.road_adjacency_matrix) == 0),'%s must be zero when road_adjacency_matrix is zero.',M_name)
end