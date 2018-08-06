function [Aeq_PaxConservation, Beq_PaxConservation] = CreateEqualityConstraintMatrices_PaxConservation(obj)
if obj.use_real_time_formulation
    warning('CreateEqualityConstraintMatrices_PaxConservation does not apply for real-time formulation.')
    Aeq_PaxConservation = [];
    Beq_PaxConservation = [];
    return;
end

n_constraint = obj.spec.N*obj.spec.M*obj.spec.C*obj.spec.Thor;

% This is meant as an upper bound for memory allocation. Unused entries are
% removed at the end.
n_constraint_entries = 2*(obj.spec.E + 2*obj.spec.NumChargers)*obj.spec.M*obj.spec.C*obj.spec.Thor + obj.spec.TotNumSources*obj.spec.C + obj.spec.M*obj.spec.Thor*obj.spec.C; 

Aeqsparse = zeros(n_constraint_entries,3);
Beq = zeros(n_constraint,1);

Aeqrow = 1;
Aeqentry = 1;

% Conservation of passengers: at every node, flow entering from other nodes
% (at appropriate charge level) equals flow exiting to other nodes (at
% appropriate charge level) plus charging flow (if applicable).
for t = 1:obj.spec.Thor
    for c = 1:obj.spec.C
        for k = 1:obj.spec.M
            for i = 1:obj.spec.N                
                if ~isempty(obj.spec.RoadGraph{i})
                    for j = obj.spec.RoadGraph{i} %Out-flows
                        if ((obj.spec.ChargeToTraverse(i,j) < c) & (t + full(obj.spec.TravelTimes(i,j)) <= obj.spec.Thor))
                            Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindRoadLinkPtckij(t,c,k,i,j), 1];
                            Aeqentry=Aeqentry+1;
                        end
                    end
                end
                if ~isempty(obj.spec.ReverseRoadGraph{i})
                    for j = obj.spec.ReverseRoadGraph{i} %In-flows
                        if (obj.spec.ChargeToTraverse(j,i) + c <= obj.spec.C & t - full(obj.spec.TravelTimes(j,i)) > 0)
                            Aeqsparse(Aeqentry,:) = [Aeqrow, obj.FindRoadLinkPtckij(t - full(obj.spec.TravelTimes(j,i)),obj.spec.ChargeToTraverse(j,i) + c,k,j,i),-1];
                            Aeqentry = Aeqentry+1;
                        end
                    end
                end
                for l = 1:length(obj.spec.ChargersList)
                    if (obj.spec.ChargersList(l) == i) %is a charger
                        if c + obj.spec.ChargerSpeed(l) <= obj.spec.C
                            %add link to i,c+1.
                            if (t + obj.spec.ChargerTime(l) <= obj.spec.Thor)
                                Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindChargeLinkPtckl(t,c,k,l),1];
                                Aeqentry = Aeqentry + 1; %Charge up to c+1, goes out
                            end
                            if (t - obj.spec.ChargerTime(l) > 0)
                                Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindDischargeLinkPtckl(t - obj.spec.ChargerTime(l),c + obj.spec.ChargerSpeed(l),k,l),-1];
                                Aeqentry = Aeqentry + 1; %Charge down from c+1, goes in
                            end
                        end
                        if (c - obj.spec.ChargerSpeed(l)) >= 1
                            if (t - obj.spec.ChargerTime(l) > 0)
                                Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindChargeLinkPtckl(t - obj.spec.ChargerTime(l),c - obj.spec.ChargerSpeed(l),k,l),-1];
                                Aeqentry=Aeqentry+1; %Charge up from c-1, goes in
                            end
                            if (t + obj.spec.ChargerTime(l) <= obj.spec.Thor)
                                Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindDischargeLinkPtckl(t,c,k,l),1];
                                Aeqentry = Aeqentry + 1; %Charge down to c-1, goes out
                            end
                        end
                    end
                end
                
                for ssi = 1:length(obj.spec.Sources{k})
                    if (obj.spec.Sources{k}(ssi) == i && obj.spec.StartTimes{k}(ssi) == t)
                        Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindPaxSourceChargecks(c,k,ssi),-1];
                        Aeqentry = Aeqentry + 1;             
                    end
                end
                if (obj.spec.Sinks(k) == i)
                    Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindPaxSinkChargetck(t,c,k),1];
                    Aeqentry = Aeqentry+1;
                    
                end
                
                %Initial conditions are handled by the rebalancers
                if t == 1
                    Beq(Aeqrow) = obj.spec.FullVehicleInitialPos(k,i,c);
                else
                    Beq(Aeqrow) = 0;
                end
                Aeqrow = Aeqrow + 1;
            end
        end
    end
end

assert(Aeqrow - 1 == n_constraint);

if (Aeqentry - 1 > n_constraint_entries)
   warning('More constraints than expected.') 
end

% Remove extra rows in Aeqsparse
Aeqsparse = Aeqsparse(1:(Aeqentry - 1),:);

Aeq_PaxConservation = sparse(Aeqsparse(:,1),Aeqsparse(:,2),Aeqsparse(:,3),n_constraint,obj.StateSize);
Beq_PaxConservation = Beq;
end












