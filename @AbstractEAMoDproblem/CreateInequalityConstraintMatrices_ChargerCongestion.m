function [Ain_ChargerCongestion, Bin_ChargerCongestion] = CreateInequalityConstraintMatrices_ChargerCongestion(obj)
% CreateInequalityConstraintMatrices_ChargerCongestion Creates inequality constraints to limit the number of vehicles that can use a charging station concurrently (Eq. 4)
%   [Ain_ChargerCongestion, Bin_ChargerCongestion] = CreateInequalityConstraintMatrices_ChargerCongestion(obj)

n_constraint = obj.spec.n_charger*obj.spec.n_time_step;

% This is meant as an upper bound for memory allocation. Unused entries are
% removed at the end.
n_constraint_entries = obj.spec.n_charger*obj.spec.n_time_step*2*(obj.n_passenger_flow_in_optimization + 1)*obj.spec.n_charge_step;

Ainsparse = zeros(n_constraint_entries,3);
Bin = zeros(n_constraint,1);

Ainrow = 1;
Ainentry = 1;

% Chargers: congestion
for t = 1:obj.spec.n_time_step
    for l = 1:obj.spec.n_charger
        for c = 1:obj.spec.n_charge_step
            % Note that if obj.n_passenger_flow_in_optimization = 0, this loop does not
            % run
            for k = 1:obj.n_passenger_flow_in_optimization
                Ainsparse(Ainentry,:) = [Ainrow,obj.FindChargeLinkPtckl(t,c,k,l),1];
                Ainentry = Ainentry + 1;
                Ainsparse(Ainentry,:) = [Ainrow,obj.FindDischargeLinkPtckl(t,c,k,l),1];
                Ainentry= Ainentry + 1;
            end
            Ainsparse(Ainentry,:) = [Ainrow,obj.FindChargeLinkRtcl(t,c,l),1];
            Ainentry = Ainentry + 1;
            Ainsparse(Ainentry,:) = [Ainrow,obj.FindDischargeLinkRtcl(t,c,l),1];
            Ainentry = Ainentry + 1;
        end
        Bin(Ainrow) = obj.spec.charger_capacity(l);
        
        Ainrow=Ainrow+1;
    end
end


assert(Ainrow - 1 == n_constraint);

if (Ainentry - 1 > n_constraint_entries)
    warning('More constraints than expected.')
end

% Remove extra rows in Ainsparse
Ainsparse = Ainsparse(1:(Ainentry - 1),:);

Ain_ChargerCongestion = sparse(Ainsparse(:,1),Ainsparse(:,2),Ainsparse(:,3),n_constraint,obj.n_state_vector);
Bin_ChargerCongestion = Bin;
end