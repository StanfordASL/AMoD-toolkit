function [Aeq_CustomerChargeConservation, Beq_CustomerChargeConservation] = CreateEqualityConstraintMatrices_CustomerChargeConservation(obj)
if ~obj.use_real_time_formulation
    warning('CreateEqualityConstraintMatrices_CustomerChargeConservation applies only for real-time formulation.')
    Aeq_CustomerChargeConservation = [];
    Beq_CustomerChargeConservation = [];
    return;
end

n_constraint = obj.spec.M*obj.spec.Thor*obj.spec.C;

% This is meant as an upper bound for memory allocation. Unused entries are
% removed at the end.
n_constraint_entries = 2*obj.spec.TotNumSources*obj.spec.C;

Aeqsparse = zeros(n_constraint_entries,3);
Beq = zeros(n_constraint,1);

Aeqrow = 1;
Aeqentry = 1;

for k = 1:obj.spec.M
    for t = 1:obj.spec.Thor
        for c = 1:obj.spec.C
            Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindPaxSinkChargetck(t,c,k),-1];
            Aeqentry = Aeqentry + 1;
            for ssi = 1 :length(obj.spec.Sources{k})
                if obj.spec.StartTimes{k}(ssi) == t - obj.RouteTime(obj.spec.Sources{k}(ssi),obj.spec.Sinks(k))
                    if c + obj.RouteCharge(obj.spec.Sources{k}(ssi),obj.spec.Sinks(k)) <= obj.spec.C && c + obj.RouteCharge(obj.spec.Sources{k}(ssi),obj.spec.Sinks(k)) > 0
                        Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindPaxSourceChargecks(c + obj.RouteCharge(obj.spec.Sources{k}(ssi),obj.spec.Sinks(k)),k,ssi),1];
                        Aeqentry=Aeqentry+1;
                    end
                end
            end
            
            Beq(Aeqrow) = 0;
            Aeqrow = Aeqrow + 1;
        end
    end
end

assert(Aeqrow - 1 == n_constraint);

if (Aeqentry - 1 > n_constraint_entries)
    warning('More constraints than expected.')
end

% Remove extra rows in Aeqsparse
Aeqsparse = Aeqsparse(1:(Aeqentry - 1),:);

Aeq_CustomerChargeConservation = sparse(Aeqsparse(:,1),Aeqsparse(:,2),Aeqsparse(:,3),n_constraint,obj.StateSize);
Beq_CustomerChargeConservation = Beq;
end