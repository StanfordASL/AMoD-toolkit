function [A_sub,B_sub] = ExtractConstraintSubmatrix(A,B,row_range,col_range)
% ExtractConstraintSubmatrix extracts the rows and columns that correspond to a constraint in a linear program

n_col = size(A,2);

A_sub = A(row_range,col_range);
B_sub = B(row_range,:);

col_range_bin = false(n_col,1);
col_range_bin(col_range) = true;

col_range = 1:n_col;
col_range_inverse = col_range(~col_range_bin);

% Verify that the constraint does not include non-zero elements that are
% outside StateSize_EAMoD
for col = col_range_inverse.'
   A_col = A(:,col);
   
   neglected_elements_col = A_col(row_range);
   
   n_nnz_neglected_elements_col = nnz(neglected_elements_col);
   
   assert(n_nnz_neglected_elements_col == 0,sprintf('Constraint is not fit for extraction at col = %d',col));
end
end