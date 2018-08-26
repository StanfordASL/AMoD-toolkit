function [route_travel_time_matrix,route_charge_to_traverse_matrix,route_travel_distance_matrix_m,route_path_cell] = BuildRoutes(obj)
% BuildRoutes Precomputes routes for passenger carrying vehicles in real-time implementation
%   [route_travel_time_matrix,route_charge_to_traverse_matrix,route_travel_distance_matrix_m,route_path_cell] = BuildRoutes(obj) where 
%   route_travel_time_matrix(i,j) is the number of time-steps needed to go from i to j,
%   route_charge_to_traverse_matrix(i,j) is the number of charge units needed to go from i to j,
%   route_travel_distance_matrix_m(i,j) is the distance in meters to go from i to j,
%   route_path_cell{i,j} is the route from i to j expresed as a vector of connected 
%   nodes that need to be traversed

route_travel_time_matrix = zeros(obj.spec.n_road_node);
route_charge_to_traverse_matrix = zeros(obj.spec.n_road_node);
route_travel_distance_matrix_m = zeros(obj.spec.n_road_node);
route_path_cell = cell(obj.spec.n_road_node,obj.spec.n_road_node);

for i = 1:obj.spec.n_road_node
    for j = 1:obj.spec.n_road_node
        route_path_cell{i,j} = FreeRouteAstar(i,j,obj.spec.road_adjacency_list,obj.spec.road_travel_time_matrix);
        
        % Account for self-loops
        if i == j
            route_travel_time_matrix(i,j) = obj.spec.road_travel_time_matrix(i,j);
            route_charge_to_traverse_matrix(i,j) = obj.spec.road_charge_to_traverse_matrix(i,j);
        else
            for k = 1:(numel(route_path_cell{i,j}) - 1)
                route_travel_time_matrix(i,j) = route_travel_time_matrix(i,j) + obj.spec.road_travel_time_matrix(route_path_cell{i,j}(k),route_path_cell{i,j}(k+1));
                route_charge_to_traverse_matrix(i,j) = route_charge_to_traverse_matrix(i,j) + obj.spec.road_charge_to_traverse_matrix(route_path_cell{i,j}(k),route_path_cell{i,j}(k+1));
                route_travel_distance_matrix_m(i,j) = route_travel_distance_matrix_m(i,j) + obj.spec.road_travel_distance_matrix_m(route_path_cell{i,j}(k),route_path_cell{i,j}(k+1));
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





