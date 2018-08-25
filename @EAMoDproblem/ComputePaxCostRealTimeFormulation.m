function pax_cost_rt = ComputePaxCostRealTimeFormulation(obj)
% ComputePaxCostRealTimeFormulation Computes the cost  associated with the pre-routed passenger-carrying vehicles in the real-time formulation
%   In the real-time formulation, passenger carrying vehicles follow pre-computed routes and are not part of the optimization.
%   See also EAMoDproblem.BuildRoutes

if obj.use_real_time_formulation
    time_cost = 0;
    distance_cost = 0;
    battery_depreciation_cost = 0;
    
    for i_sink = 1:obj.spec.NumSinks
        sink = obj.spec.Sinks(i_sink);
        
        source_array = obj.spec.Sources{i_sink};
        
        for i_source = 1:numel(source_array)
            source = source_array(i_source);
            flow = obj.spec.Flows{i_sink}(i_source);
            
            route_time = obj.RouteTime(source,sink);
            time_cost = time_cost + flow*route_time*obj.spec.ValueOfTime;
            
            route_distance = obj.RouteDistance(source,sink);            
            distance_cost = distance_cost + flow*route_distance*obj.spec.VehicleCostPerM;
            
            route_charge = obj.RouteCharge(source,sink);
            battery_depreciation_cost = battery_depreciation_cost + flow*abs(route_charge)*obj.spec.BatteryDepreciationPerUnitCharge;
        end
    end
    
    pax_cost_rt = time_cost + distance_cost;
else
    pax_cost_rt = [];
end
end