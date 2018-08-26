function [RouteTime,RouteCharge,RouteDistance,Routes] = BuildRoutes(obj)
% BuildRoutes Precomputes routes for passenger carrying vehicles in real-time implementation
%   [RouteTime,RouteCharge,RouteDistance,Routes] = BuildRoutes(obj) where 
%   RouteTime(i,j) is the number of time-steps needed to go from i to j,
%   RouteCharge(i,j) is the number of charge units needed to go from i to j,
%   RouteDistance(i,j) is the distance in meters to go from i to j,
%   Routes{i,j} is the route from i to j expresed as a vector of connected 
%   nodes that need to be traversed

RouteTime = zeros(obj.spec.n_road_node);
RouteCharge = zeros(obj.spec.n_road_node);
RouteDistance = zeros(obj.spec.n_road_node);
Routes = cell(obj.spec.n_road_node,obj.spec.n_road_node);

for i = 1:obj.spec.n_road_node
    for j = 1:obj.spec.n_road_node
        Routes{i,j} = FreeRouteAstar(i,j,obj.spec.road_adjacency_list,obj.spec.road_travel_time_matrix);
        
        % Account for self-loops
        if i == j
            RouteTime(i,j) = obj.spec.road_travel_time_matrix(i,j);
            RouteCharge(i,j) = obj.spec.road_charge_to_traverse_matrix(i,j);
        else
            for k = 1:(numel(Routes{i,j}) - 1)
                RouteTime(i,j) = RouteTime(i,j) + obj.spec.road_travel_time_matrix(Routes{i,j}(k),Routes{i,j}(k+1));
                RouteCharge(i,j) = RouteCharge(i,j) + obj.spec.road_charge_to_traverse_matrix(Routes{i,j}(k),Routes{i,j}(k+1));
                RouteDistance(i,j) = RouteDistance(i,j) + obj.spec.road_travel_distance_matrix_m(Routes{i,j}(k),Routes{i,j}(k+1));
            end
        end
    end
end

end


function [Route] = FreeRouteAstar(start,goal,road_adjacency_list,RoadCost,heuristic_est)
n_road_node=length(road_adjacency_list);
if nargin<=4
    heuristic_est = zeros(n_road_node); %revert to Dijkstra
end

%forked from the AMoD-congestion implementation

ClosedSet = [];
OpenSet = [start];

CameFrom=-1*ones(n_road_node);
g_score = Inf*ones(n_road_node,1);
f_score = Inf*ones(n_road_node,1);
g_score(start)=0;

f_score(start)=g_score(start)+heuristic_est(start,goal);

Route = [];

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
    for neigh = road_adjacency_list{currNode}
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





