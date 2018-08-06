function [Aeq_SinkConservation, Beq_SinkConservation] = CreateEqualityConstraintMatrices_BoilerPlate(obj)
n_constraint = obj.spec.TotNumSources;

% This is meant as an upper bound for memory allocation. Unused entries are
% removed at the end.
n_constraint_entries = obj.spec.TotNumSources*obj.spec.C;

Aeqsparse = zeros(n_constraint_entries,3);
Beq = zeros(n_constraint,1);

Aeqrow = 1;
Aeqentry = 1;

% FILL CODE HERE

assert(Aeqrow - 1 == n_constraint);

if (Aeqentry - 1 > n_constraint_entries)
    warning('More constraints than expected.')
end

% Remove extra rows in Aeqsparse
Aeqsparse = Aeqsparse(1:(Aeqentry - 1),:);

Aeq_SinkConservation = sparse(Aeqsparse(:,1),Aeqsparse(:,2),Aeqsparse(:,3),n_constraint,obj.StateSize);
Beq_SinkConservation = Beq;

end