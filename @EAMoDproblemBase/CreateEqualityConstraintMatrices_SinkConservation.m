function [Aeq_SinkConservation, Beq_SinkConservation] = CreateEqualityConstraintMatrices_SinkConservation(obj)
% CreateEqualityConstraintMatrices_SinkConservation Creates equality constraints to accumulate vehicles arriving at different times with different charge levels (Eq. 9c)
%   [Aeq_SinkConservation, Beq_SinkConservation] = CreateEqualityConstraintMatrices_SinkConservation(obj)



n_constraint = obj.spec.NumSinks;

% This is meant as an upper bound for memory allocation. Unused entries are
% removed at the end.
n_constraint_entries = obj.spec.NumSinks*obj.spec.C*obj.spec.Thor;

if obj.spec.sourcerelaxflag
    n_constraint_entries = n_constraint_entries + obj.spec.TotNumSources;
end

Aeqsparse = zeros(n_constraint_entries,3);
Beq = zeros(n_constraint,1);

Aeqrow = 1;
Aeqentry = 1;

% Sum of all FindPaxSinkChargeck = Pax. sink
for k = 1:obj.spec.M
    
    for c = 1:obj.spec.C
        for t = 1:obj.spec.Thor
            Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindPaxSinkChargetck(t,c,k),1];
            Aeqentry = Aeqentry+1;
        end
    end
    if obj.spec.sourcerelaxflag
        for ssi = 1:length(obj.spec.Sources{k})
            Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindSourceRelaxks(k,ssi),1];
            Aeqentry = Aeqentry+1;
        end
    end
    
    Beq(Aeqrow) = sum(obj.spec.Flows{k});
    
    Aeqrow = Aeqrow+1;
end

assert(Aeqrow - 1 == n_constraint);

if (Aeqentry - 1 > n_constraint_entries)
    warning('More constraints than expected.')
end

% Remove extra rows in Aeqsparse
Aeqsparse = Aeqsparse(1:(Aeqentry - 1),:);

Aeq_SinkConservation = sparse(Aeqsparse(:,1),Aeqsparse(:,2),Aeqsparse(:,3),n_constraint,obj.StateSize);
Beq_SinkConservation = Beq;

end