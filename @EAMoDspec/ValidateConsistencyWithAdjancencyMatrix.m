function ValidateConsistencyWithAdjancencyMatrix(obj,M,varargin)
% ValidateConsistencyWithAdjancencyMatrix asserts that a matrix M
% has the same size as adjacency_matrix and that M is zero when
% adjacency_matrix is zero.
% ValidateConsistencyWithAdjancencyMatrix(obj,M)
% ValidateConsistencyWithAdjancencyMatrix(obj,M,M_name) where
% M_name is the name of M to be included in error messages

n_varargin = numel(varargin);

switch n_varargin
    case 0
        M_name = 'M';
    case 1
        M_name = varargin{1};
        
    otherwise
        error('Too many arguments')
end

assert(all(size(M) == size(obj.adjacency_matrix)),'Size of %s must match that of adjacency_matrix.',M_name)
assert(all(M(~obj.adjacency_matrix) == 0),'%s must be zero when adjacency_matrix is zero.',M_name)
end