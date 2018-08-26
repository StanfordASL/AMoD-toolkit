function [Ain_RoadCongestion, Bin_RoadCongestion] = CreateInequalityConstraintMatrices_RoadCongestion(obj)
% CreateInequalityConstraintMatrices_RoadCongestion Creates inequality constraints to limit the flow along the road edges to be less than their capacity (Eq. 3) 
%   [Ain_RoadCongestion, Bin_RoadCongestion] = CreateInequalityConstraintMatrices_RoadCongestion(obj)

n_constraint = obj.spec.n_road_edge*obj.spec.n_time_step;

% This is meant as an upper bound for memory allocation. Unused entries are
% removed at the end.
n_constraint_entries = obj.spec.n_road_edge*obj.spec.n_time_step*(obj.n_passenger_flow_in_optimization + 1)*obj.spec.n_charge_step;

Ainsparse = zeros(n_constraint_entries,3);
Bin = zeros(n_constraint,1);

Ainrow = 1;
Ainentry = 1;

% Roads: congestion
for t = 1:obj.spec.n_time_step
    for i = 1:obj.spec.n_road_node
        for j = obj.spec.road_adjacency_list{i}
            for c = 1:obj.spec.n_charge_step
                % Note that if obj.n_passenger_flow_in_optimization = 0, this loop does not
                % run
                for k = 1:obj.n_passenger_flow_in_optimization
                    Ainsparse(Ainentry,:) = [Ainrow,obj.FindRoadLinkPtckij(t,c,k,i,j),1];
                    Ainentry = Ainentry+1;
                end
                Ainsparse(Ainentry,:) = [Ainrow,obj.FindRoadLinkRtcij(t,c,i,j),1];
                Ainentry = Ainentry + 1;
            end
            
            Bin(Ainrow) = obj.road_residual_capacity_matrix(i,j,t);
            
            Ainrow = Ainrow + 1;
        end
    end
end


assert(Ainrow - 1 == n_constraint);

if (Ainentry - 1 > n_constraint_entries)
    warning('More constraints than expected.')
end

% Remove extra rows in Ainsparse
Ainsparse = Ainsparse(1:(Ainentry - 1),:);

Ain_RoadCongestion = sparse(Ainsparse(:,1),Ainsparse(:,2),Ainsparse(:,3),n_constraint,obj.n_state_vector);
Bin_RoadCongestion = Bin;

end