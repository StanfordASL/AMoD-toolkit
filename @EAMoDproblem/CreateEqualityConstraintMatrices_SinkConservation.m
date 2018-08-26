function [Aeq_SinkConservation, Beq_SinkConservation] = CreateEqualityConstraintMatrices_SinkConservation(obj)
% CreateEqualityConstraintMatrices_SinkConservation Creates equality constraints to accumulate vehicles arriving at different times with different charge levels (Eq. 9c)
%   [Aeq_SinkConservation, Beq_SinkConservation] = CreateEqualityConstraintMatrices_SinkConservation(obj)

n_constraint = obj.spec.n_passenger_sink;

% This is meant as an upper bound for memory allocation. Unused entries are
% removed at the end.
n_constraint_entries = obj.spec.n_passenger_sink*obj.spec.n_charge_step*obj.spec.n_time_step;

if obj.sourcerelaxflag
    n_constraint_entries = n_constraint_entries + obj.spec.n_passenger_source;
end

Aeqsparse = zeros(n_constraint_entries,3);
Beq = zeros(n_constraint,1);

Aeqrow = 1;
Aeqentry = 1;

% Sum of all FindPaxSinkChargeck = Pax. sink
for k = 1:obj.spec.n_passenger_flow
    
    for c = 1:obj.spec.n_charge_step
        for t = 1:obj.spec.n_time_step
            Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindPaxSinkChargetck(t,c,k),1];
            Aeqentry = Aeqentry+1;
        end
    end
    if obj.sourcerelaxflag
        for ssi = 1:length(obj.spec.passenger_source_list_cell{k})
            Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindSourceRelaxks(k,ssi),1];
            Aeqentry = Aeqentry+1;
        end
    end
    
    Beq(Aeqrow) = sum(obj.spec.passenger_flow_list_cell{k});
    
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