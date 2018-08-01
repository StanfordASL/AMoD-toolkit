function [Aeq_SourceConservation, Beq_SourceConservation] = CreateEqualityConstraintMatrices_SourceConservation(obj)

n_constraint = obj.TotNumSources;

% This is meant as an upper bound for memory allocation. Unused entries are
% removed at the end.
n_constraint_entries = obj.TotNumSources*obj.C;

if obj.sourcerelaxflag
    n_constraint_entries = n_constraint_entries + obj.TotNumSources;
end

Aeqsparse = zeros(n_constraint_entries,3);
Beq = zeros(n_constraint,1);

Aeqrow = 1;
Aeqentry = 1;


% Sum of all FindPaxSourceChargeck = Pax. source
for k = 1:obj.M
    for ssi = 1:length(obj.Sources{k})
        for c = 1:obj.C
            Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindPaxSourceChargecks(c,k,ssi),1];
            Aeqentry = Aeqentry + 1;
        end
        if obj.sourcerelaxflag
            Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindSourceRelaxks(k,ssi),1];
            Aeqentry = Aeqentry + 1;
        end
        
        Beq(Aeqrow) = obj.Flows{k}(ssi);
        Aeqrow = Aeqrow+1;
    end
end

assert(Aeqrow - 1 == n_constraint);

if (Aeqentry - 1 > n_constraint_entries)
    warning('More constraints than expected.')
end

% Remove extra rows in Aeqsparse
Aeqsparse = Aeqsparse(1:(Aeqentry - 1),:);

Aeq_SourceConservation = sparse(Aeqsparse(:,1),Aeqsparse(:,2),Aeqsparse(:,3),n_constraint,obj.StateSize);
Beq_SourceConservation = Beq;
end