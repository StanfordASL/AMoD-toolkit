function SetOffsetsForIndexingFunctions(obj)
% SetOffsetsForIndexingFunctions Sets the cached offset variables for indexer functions

obj.offset_FindChargeLinkHelpertckij_t = zeros(obj.spec.n_time_step,1);
obj.offset_FindDischargeLinkHelpertckl_t = zeros(obj.spec.n_time_step,1);

for t = 1:obj.spec.n_time_step
    obj.offset_FindChargeLinkHelpertckij_t(t) = obj.FindRoadLinkRtcij(t,obj.spec.n_charge_step,obj.spec.n_road_node,obj.spec.road_adjacency_list{end}(end));
    obj.offset_FindDischargeLinkHelpertckl_t(t) = obj.FindChargeLinkRtcl(t,obj.spec.n_charge_step,obj.spec.n_charger);
    
end

obj.offset_FindPaxSourceChargecks = obj.FindDischargeLinkRtcl(obj.spec.n_time_step,obj.spec.n_charge_step,obj.spec.n_charger);
obj.offset_FindPaxSinkChargetck =  obj.FindPaxSourceChargecks(obj.spec.n_charge_step,obj.spec.n_passenger_flow,obj.spec.n_sources_to_sink(end));

obj.offset_FindEndRebLocationci = obj.FindPaxSinkChargetck(obj.spec.n_time_step,obj.spec.n_charge_step,obj.spec.n_passenger_flow);
end