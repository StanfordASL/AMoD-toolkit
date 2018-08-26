function [Aeq_CustomerChargeConservation, Beq_CustomerChargeConservation] = CreateEqualityConstraintMatrices_CustomerChargeConservation(obj)
% CreateEqualityConstraintMatrices_CustomerChargeConservation Creates equality constraints for conservation of charge in customer-carrying vehicles in the real-time formulation (Eq. 11)
%   [Aeq_CustomerChargeConservation, Beq_CustomerChargeConservation] = CreateEqualityConstraintMatrices_CustomerChargeConservation


if ~obj.use_real_time_formulation
    warning('CreateEqualityConstraintMatrices_CustomerChargeConservation applies only for real-time formulation.')
    Aeq_CustomerChargeConservation = [];
    Beq_CustomerChargeConservation = [];
    return;
end

n_constraint = obj.spec.n_passenger_flow*obj.spec.n_time_step*obj.spec.n_charge_step;

% This is meant as an upper bound for memory allocation. Unused entries are
% removed at the end.
n_constraint_entries = 2*obj.spec.n_passenger_source*obj.spec.n_charge_step;

Aeqsparse = zeros(n_constraint_entries,3);
Beq = zeros(n_constraint,1);

Aeqrow = 1;
Aeqentry = 1;

for k = 1:obj.spec.n_passenger_flow
    for t = 1:obj.spec.n_time_step
        for c = 1:obj.spec.n_charge_step
            Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindPaxSinkChargetck(t,c,k),-1];
            Aeqentry = Aeqentry + 1;
            for ssi = 1 :length(obj.spec.passenger_source_list_cell{k})
                if obj.spec.passenger_start_time_list_cell{k}(ssi) == t - obj.RouteTime(obj.spec.passenger_source_list_cell{k}(ssi),obj.spec.passenger_sink_list(k))
                    if c + obj.RouteCharge(obj.spec.passenger_source_list_cell{k}(ssi),obj.spec.passenger_sink_list(k)) <= obj.spec.n_charge_step && c + obj.RouteCharge(obj.spec.passenger_source_list_cell{k}(ssi),obj.spec.passenger_sink_list(k)) > 0
                        Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindPaxSourceChargecks(c + obj.RouteCharge(obj.spec.passenger_source_list_cell{k}(ssi),obj.spec.passenger_sink_list(k)),k,ssi),1];
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