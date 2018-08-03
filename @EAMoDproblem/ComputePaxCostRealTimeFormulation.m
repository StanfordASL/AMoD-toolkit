function pax_cost_rt = ComputePaxCostRealTimeFormulation(obj)
if obj.use_real_time_formulation    
    time_cost = 0;
    distance_cost = 0;
    
    for i_sink = 1:obj.spec.NumSinks
        sink = obj.spec.Sinks(i_sink);
        
        source_array = obj.spec.Sources{i_sink};
                
        for i_source = 1:numel(source_array)
            source = source_array(i_source);
            flow = obj.spec.Flows{i_sink}(i_source);
            
            route_time = obj.RouteTime(source,sink);
            
            route_distance = obj.RouteDistance(source,sink);    
            
            time_cost = time_cost + flow*route_time*obj.spec.ValueOfTime;
            
            distance_cost = distance_cost + flow*route_distance*obj.spec.VehicleCostPerM;
        end
    end    

    pax_cost_rt = time_cost + distance_cost;
else
    pax_cost_rt = [];
end
end