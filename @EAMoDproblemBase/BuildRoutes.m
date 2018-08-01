function [RouteTime,RouteCharge,RouteDistance,Routes] = BuildRoutes(obj)
RouteTime = zeros(obj.N);
RouteCharge = zeros(obj.N);
RouteDistance = zeros(obj.N);
Routes = cell(obj.N,obj.N);

for i = 1:obj.N
    for j = 1:obj.N
        Routes{i,j} = FreeRouteAstar(i,j,obj.RoadGraph,obj.TravelTimes);
        
        % Account for self-loops
        if i == j
            RouteTime(i,j) = obj.TravelTimes(i,j);
            RouteCharge(i,j) = obj.ChargeToTraverse(i,j);
        else
            for k = 1:(numel(Routes{i,j}) - 1)
                RouteTime(i,j) = RouteTime(i,j) + obj.TravelTimes(Routes{i,j}(k),Routes{i,j}(k+1));
                RouteCharge(i,j) = RouteCharge(i,j) + obj.ChargeToTraverse(Routes{i,j}(k),Routes{i,j}(k+1));
                RouteDistance(i,j) = RouteDistance(i,j) + obj.TravelDistance(Routes{i,j}(k),Routes{i,j}(k+1));
            end
        end
    end    
end

end


function [Route] = FreeRouteAstar(start,goal,RoadGraph,RoadCost,heuristic_est)
N=length(RoadGraph);
if nargin<=4
    heuristic_est = zeros(N); %revert to Dijkstra
end

%forked from the AMoD-congestion implementation

ClosedSet = [];
OpenSet = [start];

CameFrom=-1*ones(N);
g_score = Inf*ones(N,1);
f_score = Inf*ones(N,1);
g_score(start)=0;

f_score(start)=g_score(start)+heuristic_est(start,goal);

while ~isempty(OpenSet)
    [~,currInOpenSet]=min(f_score(OpenSet));
    currNode = OpenSet(currInOpenSet);
    if currNode == goal
        
        
        Route = goal;
        while (Route(1)~=start)
            Route=[CameFrom(Route(1)),Route];
        end
        
        return
    end
    OpenSet = OpenSet(OpenSet~=currNode);       %Remove current node
    ClosedSet = [ClosedSet, currNode];          %Add to closed set
    for neigh = RoadGraph{currNode}
        if ~sum(ClosedSet == neigh)             %If neighbor is not in closed set
            tentative_g = g_score(currNode) + RoadCost(currNode,neigh);
            if (~sum(neigh == OpenSet))         %If the neighbor is not already in OpenSet, add it
                OpenSet = [OpenSet, neigh];     
            elseif tentative_g>=g_score(neigh)
                continue
            end
            CameFrom(neigh) = currNode;
            g_score(neigh) = tentative_g;
            f_score(neigh) = g_score(neigh)+heuristic_est(neigh,goal);
            
        end
    end
end

end





