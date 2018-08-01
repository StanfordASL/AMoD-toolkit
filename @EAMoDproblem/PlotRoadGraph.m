function PlotRoadGraph(obj)

G = digraph(obj.TravelDistance);

p = plot(G,'Layout','layered','EdgeLabel',G.Edges.Weight,'ArrowSize',15,'LineWidth',2);

highlight(p,obj.ChargersList,'NodeColor','r')

end