% Similar to verifyEqual but exploits sparsity. Note that verifyEqual fails
% for large arrays.
function verifyEqualSparse(test_case,S,S_ref,varargin)
S_delta = S - S_ref;

[row,col,values] = find(S_delta);

if ~isempty(values)
    verifyEqual(test_case,values,zeros(size(values)),varargin{:});
end
end