function figure_handle = PlotRoadGraph(obj)
% PlotRoadGraph Creates a plot with obj.RoadGraph. Displays distance for road edges and nodes with charging stations are shown in red.
figure_handle = figure;
hold on

node_marker_size = 5;

charger_color = 'r';
% This plot is needed to get the legend right
plot(nan,nan,['o',charger_color],'MarkerSize',node_marker_size,'MarkerFaceColor',charger_color)

% Round for nice display
travel_distance_rounded_km = round(obj.spec.TravelDistance/1e3,2);

G = digraph(travel_distance_rounded_km);

p = plot(G,'Layout','layered','EdgeLabel',G.Edges.Weight,'ArrowSize',15,'LineWidth',2,'MarkerSize',node_marker_size);

highlight(p,obj.spec.ChargersList,'NodeColor',charger_color)

legend('Nodes with charging station','Graph showing road length in km','Location','best')
end