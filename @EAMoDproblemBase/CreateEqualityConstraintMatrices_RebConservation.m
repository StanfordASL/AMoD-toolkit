function [Aeq_RebConservation, Beq_RebConservation] = CreateEqualityConstraintMatrices_RebConservation(obj)
% CreateEqualityConstraintMatrices_RebConservation Creates equality constraints for conservation of rebalancing flow, and enforcement of initial and final conditions (Eq. 2)
%   [Aeq_RebConservation, Beq_RebConservation] = CreateEqualityConstraintMatrices_RebConservation(obj)

n_constraint = obj.spec.N*obj.spec.C*obj.spec.Thor;

% This is meant as an upper bound for memory allocation. Unused entries are
% removed at the end.
n_constraint_entries = 2*(obj.spec.E + 2*obj.spec.NumChargers)*obj.spec.C*obj.spec.Thor + obj.spec.TotNumSources*obj.spec.C + obj.spec.M*obj.spec.Thor*obj.spec.C;

Aeqsparse = zeros(n_constraint_entries,3);
Beq = zeros(n_constraint,1);

Aeqrow = 1;
Aeqentry = 1;

% Conservation of rebalancers
for t = 1:obj.spec.Thor
    for c = 1:obj.spec.C
        for i = 1:obj.spec.N
            if ~isempty(obj.spec.RoadGraph{i})
                for j = obj.spec.RoadGraph{i} %Out-flows
                    if ((obj.spec.ChargeToTraverse(i,j)< c) && (t + full(obj.spec.TravelTimes(i,j)) <= obj.spec.Thor))
                        Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindRoadLinkRtcij(t,c,i,j), 1];
                        Aeqentry = Aeqentry + 1;
                    end
                end
            end
            if ~isempty(obj.spec.ReverseRoadGraph{i})
                for j = obj.spec.ReverseRoadGraph{i} %In-flows
                    if (obj.spec.ChargeToTraverse(j,i) + c <= obj.spec.C && t - full(obj.spec.TravelTimes(j,i))>0)
                        Aeqsparse(Aeqentry,:)=[Aeqrow,obj.FindRoadLinkRtcij(t-full(obj.spec.TravelTimes(j,i)),obj.spec.ChargeToTraverse(j,i)+c,j,i),-1];
                        Aeqentry=Aeqentry+1;
                    end
                end
            end
            
            for l = 1:length(obj.spec.ChargersList)
                if (obj.spec.ChargersList(l) == i) %is a charger
                    if c + obj.spec.ChargerSpeed(l) <= obj.spec.C
                        %add link to i,c+1.
                        if (t + obj.spec.ChargerTime(l) <= obj.spec.Thor)
                            Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindChargeLinkRtcl(t,c,l),1];
                            Aeqentry = Aeqentry+1; %Charge up to c+1, goes out
                        end
                        if (t - obj.spec.ChargerTime(l) > 0)
                            Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindDischargeLinkRtcl(t - obj.spec.ChargerTime(l),c + obj.spec.ChargerSpeed(l),l),-1];
                            Aeqentry = Aeqentry+1; %Charge down from c+1, goes in
                        end
                    end
                    if c - obj.spec.ChargerSpeed(l) >= 1
                        if (t - obj.spec.ChargerTime(l) > 0)
                            Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindChargeLinkRtcl(t - obj.spec.ChargerTime(l),c - obj.spec.ChargerSpeed(l),l),-1];
                            Aeqentry = Aeqentry + 1; %Charge up from c-1, goes in
                        end
                        if (t + obj.spec.ChargerTime(l) <= obj.spec.Thor)
                            Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindDischargeLinkRtcl(t,c,l),1];
                            Aeqentry = Aeqentry+1; %Charge down to c-1, goes out
                        end
                    end
                end
            end
            
            for k = 1:obj.spec.M
                for ssi = 1:length(obj.spec.Sources{k})
                    if (obj.spec.Sources{k}(ssi) == i && obj.spec.StartTimes{k}(ssi) == t)
                        Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindPaxSourceChargecks(c,k,ssi),1];
                        Aeqentry = Aeqentry+1;
                        % Departing passengers (exiting vehicles) at charge level c
                    end
                end
                if (obj.spec.Sinks(k) == i)
                    Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindPaxSinkChargetck(t,c,k),-1];
                    Aeqentry = Aeqentry+1;
                    %Arriving passengers (entering vehicles)
                end
                
            end
            
            % Final conditions
            if t == obj.spec.Thor
                Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindEndRebLocationci(c,i),1];
                Aeqentry = Aeqentry + 1;
            end
            
            % Initial conditions!
            if t==1
                Beq(Aeqrow) = obj.spec.EmptyVehicleInitialPos(i,c);
            else
                Beq(Aeqrow) = 0;
            end
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

Aeq_RebConservation = sparse(Aeqsparse(:,1),Aeqsparse(:,2),Aeqsparse(:,3),n_constraint,obj.StateSize);
Beq_RebConservation = Beq;
end