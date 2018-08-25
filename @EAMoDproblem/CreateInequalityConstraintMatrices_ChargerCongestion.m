function [Ain_ChargerCongestion, Bin_ChargerCongestion] = CreateInequalityConstraintMatrices_ChargerCongestion(obj)
% CreateInequalityConstraintMatrices_ChargerCongestion Creates inequality constraints to limit the number of vehicles that can use a charging station concurrently (Eq. 4)
%   [Ain_ChargerCongestion, Bin_ChargerCongestion] = CreateInequalityConstraintMatrices_ChargerCongestion(obj)

n_constraint = obj.spec.NumChargers*obj.spec.Thor;

% This is meant as an upper bound for memory allocation. Unused entries are
% removed at the end.
n_constraint_entries = obj.spec.NumChargers*obj.spec.Thor*2*(obj.num_passenger_flows + 1)*obj.spec.C;

Ainsparse = zeros(n_constraint_entries,3);
Bin = zeros(n_constraint,1);

Ainrow = 1;
Ainentry = 1;

% Chargers: congestion
for t = 1:obj.spec.Thor
    for l = 1:obj.spec.NumChargers
        for c = 1:obj.spec.C
            % Note that if obj.num_passenger_flows = 0, this loop does not
            % run
            for k = 1:obj.num_passenger_flows
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
        Bin(Ainrow) = obj.spec.ChargerCap(l);
        
        Ainrow=Ainrow+1;
    end
end


assert(Ainrow - 1 == n_constraint);

if (Ainentry - 1 > n_constraint_entries)
    warning('More constraints than expected.')
end

% Remove extra rows in Ainsparse
Ainsparse = Ainsparse(1:(Ainentry - 1),:);

Ain_ChargerCongestion = sparse(Ainsparse(:,1),Ainsparse(:,2),Ainsparse(:,3),n_constraint,obj.StateSize);
Bin_ChargerCongestion = Bin;
end