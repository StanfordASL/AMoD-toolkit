function [Ain_RoadCongestion, Bin_RoadCongestion] = CreateInequalityConstraintMatrices_RoadCongestion(obj)
% CreateInequalityConstraintMatrices_RoadCongestion Creates inequality constraints to limit the flow along the road edges to be less than their capacity (Eq. 3) 
%   [Ain_RoadCongestion, Bin_RoadCongestion] = CreateInequalityConstraintMatrices_RoadCongestion(obj)

if obj.use_real_time_formulation
   ExpandedRoadCap = repmat(full(obj.spec.RoadCap),1,1,obj.spec.Thor);
   residual_road_cap_relative = obj.TVRoadCap./ExpandedRoadCap;

    if any(residual_road_cap_relative(:) < 0)
        [min_residual_road_cap_relative,index] = min(residual_road_cap_relative(:));
        [i,j,t] = ind2sub(size(obj.TVRoadCap),index);

        warning('Residual road capacity is negative. Real-time formulation will not be feasible. Maximal excedent: %.2f %% in edge %i-%i at time-step %i.',-min_residual_road_cap_relative*100,i,j,t)   
    end 
    
end

n_constraint = obj.spec.E*obj.spec.Thor;

% This is meant as an upper bound for memory allocation. Unused entries are
% removed at the end.
n_constraint_entries = obj.spec.E*obj.spec.Thor*(obj.num_passenger_flows + 1)*obj.spec.C;

Ainsparse = zeros(n_constraint_entries,3);
Bin = zeros(n_constraint,1);

Ainrow = 1;
Ainentry = 1;

% Roads: congestion
for t = 1:obj.spec.Thor
    for i = 1:obj.spec.N
        for j = obj.spec.RoadGraph{i}
            for c = 1:obj.spec.C
                % Note that if obj.num_passenger_flows = 0, this loop does not
                % run
                for k = 1:obj.num_passenger_flows
                    Ainsparse(Ainentry,:) = [Ainrow,obj.FindRoadLinkPtckij(t,c,k,i,j),1];
                    Ainentry = Ainentry+1;
                end
                Ainsparse(Ainentry,:) = [Ainrow,obj.FindRoadLinkRtcij(t,c,i,j),1];
                Ainentry = Ainentry + 1;
            end
            
            if obj.use_real_time_formulation
                Bin(Ainrow) = obj.TVRoadCap(i,j,t);
            else
                Bin(Ainrow) = obj.spec.RoadCap(i,j);
            end
            
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

Ain_RoadCongestion = sparse(Ainsparse(:,1),Ainsparse(:,2),Ainsparse(:,3),n_constraint,obj.StateSize);
Bin_RoadCongestion = Bin;

end