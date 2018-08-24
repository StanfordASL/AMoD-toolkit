%Based on https://stackoverflow.com/questions/5243890/how-to-get-a-diagonal-matrix-from-a-vector
function D = spdiag(vec)
    n = length(vec);
    D = spdiags(vec(:),0,n,n);
end