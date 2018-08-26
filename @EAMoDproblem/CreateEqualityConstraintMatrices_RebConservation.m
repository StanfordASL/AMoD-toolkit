function [Aeq_RebConservation, Beq_RebConservation] = CreateEqualityConstraintMatrices_RebConservation(obj)
% CreateEqualityConstraintMatrices_RebConservation Creates equality constraints for conservation of rebalancing flow, and enforcement of initial and final conditions (Eq. 2)
%   [Aeq_RebConservation, Beq_RebConservation] = CreateEqualityConstraintMatrices_RebConservation(obj)

n_constraint = obj.spec.n_road_node*obj.spec.n_charge_step*obj.spec.n_time_step;

% This is meant as an upper bound for memory allocation. Unused entries are
% removed at the end.
n_constraint_entries = 2*(obj.spec.n_road_edge + 2*obj.spec.n_charger)*obj.spec.n_charge_step*obj.spec.n_time_step + obj.spec.n_passenger_source*obj.spec.n_charge_step + obj.spec.n_passenger_flow*obj.spec.n_time_step*obj.spec.n_charge_step;

Aeqsparse = zeros(n_constraint_entries,3);
Beq = zeros(n_constraint,1);

Aeqrow = 1;
Aeqentry = 1;

% Conservation of rebalancers
for t = 1:obj.spec.n_time_step
    for c = 1:obj.spec.n_charge_step
        for i = 1:obj.spec.n_road_node
            if ~isempty(obj.spec.road_adjacency_list{i})
                for j = obj.spec.road_adjacency_list{i} %Out-flows
                    if ((obj.spec.road_charge_to_traverse_matrix(i,j)< c) && (t + full(obj.spec.road_travel_time_matrix(i,j)) <= obj.spec.n_time_step))
                        Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindRoadLinkRtcij(t,c,i,j), 1];
                        Aeqentry = Aeqentry + 1;
                    end
                end
            end
            if ~isempty(obj.spec.road_reverse_adjacency_list{i})
                for j = obj.spec.road_reverse_adjacency_list{i} %In-flows
                    if (obj.spec.road_charge_to_traverse_matrix(j,i) + c <= obj.spec.n_charge_step && t - full(obj.spec.road_travel_time_matrix(j,i))>0)
                        Aeqsparse(Aeqentry,:)=[Aeqrow,obj.FindRoadLinkRtcij(t-full(obj.spec.road_travel_time_matrix(j,i)),obj.spec.road_charge_to_traverse_matrix(j,i)+c,j,i),-1];
                        Aeqentry=Aeqentry+1;
                    end
                end
            end
            
            for l = 1:length(obj.spec.charger_list)
                if (obj.spec.charger_list(l) == i) %is a charger
                    if c + obj.spec.charger_speed(l) <= obj.spec.n_charge_step
                        %add link to i,c+1.
                        if (t + obj.spec.charger_time(l) <= obj.spec.n_time_step)
                            Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindChargeLinkRtcl(t,c,l),1];
                            Aeqentry = Aeqentry+1; %Charge up to c+1, goes out
                        end
                        if (t - obj.spec.charger_time(l) > 0)
                            Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindDischargeLinkRtcl(t - obj.spec.charger_time(l),c + obj.spec.charger_speed(l),l),-1];
                            Aeqentry = Aeqentry+1; %Charge down from c+1, goes in
                        end
                    end
                    if c - obj.spec.charger_speed(l) >= 1
                        if (t - obj.spec.charger_time(l) > 0)
                            Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindChargeLinkRtcl(t - obj.spec.charger_time(l),c - obj.spec.charger_speed(l),l),-1];
                            Aeqentry = Aeqentry + 1; %Charge up from c-1, goes in
                        end
                        if (t + obj.spec.charger_time(l) <= obj.spec.n_time_step)
                            Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindDischargeLinkRtcl(t,c,l),1];
                            Aeqentry = Aeqentry+1; %Charge down to c-1, goes out
                        end
                    end
                end
            end
            
            for k = 1:obj.spec.n_passenger_flow
                for ssi = 1:length(obj.spec.passenger_source_list_cell{k})
                    if (obj.spec.passenger_source_list_cell{k}(ssi) == i && obj.spec.passenger_start_time_list_cell{k}(ssi) == t)
                        Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindPaxSourceChargecks(c,k,ssi),1];
                        Aeqentry = Aeqentry+1;
                        % Departing passengers (exiting vehicles) at charge level c
                    end
                end
                if (obj.spec.passenger_sink_list(k) == i)
                    Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindPaxSinkChargetck(t,c,k),-1];
                    Aeqentry = Aeqentry+1;
                    %Arriving passengers (entering vehicles)
                end
                
            end
            
            % Final conditions
            if t == obj.spec.n_time_step
                Aeqsparse(Aeqentry,:) = [Aeqrow,obj.FindEndRebLocationci(c,i),1];
                Aeqentry = Aeqentry + 1;
            end
            
            % Initial conditions!
            if t==1
                Beq(Aeqrow) = obj.spec.initial_state_empty_vehicles(i,c);
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