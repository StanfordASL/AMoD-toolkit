function [Ain_RoadCongestion, Bin_RoadCongestion] = CreateInequalityConstraintMatrices_RoadCongestion(obj)
n_constraint = obj.E*obj.Thor;

% This is meant as an upper bound for memory allocation. Unused entries are
% removed at the end.
n_constraint_entries = obj.E*obj.Thor*(obj.num_passenger_flows + 1)*obj.C;  

Ainsparse = zeros(n_constraint_entries,3);
Bin = zeros(n_constraint,1);

Ainrow = 1;
Ainentry = 1;

% Roads: congestion
for t = 1:obj.Thor
    for i = 1:obj.N
        for j = obj.RoadGraph{i}
            for c = 1:obj.C
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
                Bin(Ainrow) = obj.TVRoadCap(t,i,j);
            else                
                Bin(Ainrow) = obj.RoadCap(i,j);
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