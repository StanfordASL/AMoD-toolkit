function PlotRoadGraph(obj)

G = digraph(obj.spec.TravelDistance);

p = plot(G,'Layout','layered','EdgeLabel',G.Edges.Weight,'ArrowSize',15,'LineWidth',2);

highlight(p,obj.spec.ChargersList,'NodeColor','r')

end