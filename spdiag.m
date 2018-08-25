function D = spdiag(vec)
% spdiag Returns a sparse matrix with the entries of vec in its diagonal
%   D = spdiag(vec) where vec is a vector and D a sparse matrix
%   Based on https://stackoverflow.com/questions/5243890/how-to-get-a-diagonal-matrix-from-a-vector

    n = numel(vec);
    D = spdiags(vec(:),0,n,n);
end