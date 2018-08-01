function [Aeq_RebConservation, Beq_RebConservation] = CreateEqualityConstraintMatrices_RebConservation(obj)

n_constraint = obj.N*obj.C*obj.Thor;

% This is meant as an upper bound for memory allocation. Unused entries are
% removed at the end.
n_constraint_entries = 2*(obj.E + 2*obj.NumChargers)*obj.C*obj.Thor + obj.TotNumSources*obj.C + obj.M*obj.Thor*obj.C; 

Aeqsparse = zeros(n_constraint_entries,3);
Beq = zeros(n_constraint,1);

Aeqrow = 1;
Aeqentry = 1;

% Conservation of rebalancers
for t = 1:obj.Thor
    for c = 1:obj.C
        for i = 1:obj.N            
            if ~isempty(obj.RoadGraph{i})
                for j = obj.RoadGraph{i} %Out-flows
                    if ((obj.ChargeToTraverse(i,j)< c) && (t + full(obj.TravelTimes(i,j)) <= obj.Thor))
                        Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindRoadLinkRtcij(t,c,i,j), 1];
                        Aeqentry = Aeqentry + 1;
                    end
                end
            end
            if ~isempty(obj.ReverseRoadGraph{i})
                for j = obj.ReverseRoadGraph{i} %In-flows
                    if (obj.ChargeToTraverse(j,i) + c <= obj.C && t - full(obj.TravelTimes(j,i))>0)
                        Aeqsparse(Aeqentry,:)=[Aeqrow,obj.FindRoadLinkRtcij(t-full(obj.TravelTimes(j,i)),obj.ChargeToTraverse(j,i)+c,j,i),-1];
                        Aeqentry=Aeqentry+1;
                    end
                end
            end
            
            for l = 1:length(obj.ChargersList)
                if (obj.ChargersList(l) == i) %is a charger
                    if c + obj.ChargerSpeed(l) <= obj.C
                        %add link to i,c+1.
                        if (t + obj.ChargerTime(l) <= obj.Thor)
                            Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindChargeLinkRtcl(t,c,l),1];
                            Aeqentry = Aeqentry+1; %Charge up to c+1, goes out
                        end
                        if (t - obj.ChargerTime(l) > 0)
                            Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindDischargeLinkRtcl(t - obj.ChargerTime(l),c + obj.ChargerSpeed(l),l),-1];
                            Aeqentry = Aeqentry+1; %Charge down from c+1, goes in
                        end
                    end
                    if c - obj.ChargerSpeed(l) >= 1
                        if (t - obj.ChargerTime(l) > 0)
                            Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindChargeLinkRtcl(t - obj.ChargerTime(l),c - obj.ChargerSpeed(l),l),-1];
                            Aeqentry = Aeqentry + 1; %Charge up from c-1, goes in
                        end
                        if (t + obj.ChargerTime(l) <= obj.Thor)
                            Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindDischargeLinkRtcl(t,c,l),1];
                            Aeqentry = Aeqentry+1; %Charge down to c-1, goes out
                        end
                    end
                end
            end
            
            for k = 1:obj.M
                for ssi = 1:length(obj.Sources{k})
                    if (obj.Sources{k}(ssi) == i && obj.StartTimes{k}(ssi) == t)
                        Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindPaxSourceChargecks(c,k,ssi),1];
                        Aeqentry = Aeqentry+1;
                        % Departing passengers (exiting vehicles) at charge level c
                    end
                end
                if (obj.Sinks(k) == i)
                    Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindPaxSinkChargetck(t,c,k),-1];
                    Aeqentry = Aeqentry+1;
                    %Arriving passengers (entering vehicles)
                end
                
            end
            
            % Final conditions
            if t == obj.Thor
                Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindEndRebLocationci(c,i),1];
                Aeqentry = Aeqentry + 1;
            end
            
            % Initial conditions!
            if t==1
                Beq(Aeqrow) = obj.EmptyVehicleInitialPos(i,c);
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