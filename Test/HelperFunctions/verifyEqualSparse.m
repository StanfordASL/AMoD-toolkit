function verifyEqualSparse(test_case,S,S_ref,varargin)
% verifyEqualSparse Verifies the equality of a value to a reference exploiting sparsity
%   verifyEqual often fails for large arrays
%
%   See also verifyEqual


S_delta = S - S_ref;

[row,col,values] = find(S_delta);

if ~isempty(values)
    verifyEqual(test_case,values,zeros(size(values)),varargin{:});
end
end