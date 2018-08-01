% Energy balance:
% E_start + E_charge = E_trip + E_end
function [E_charge_min_j,P_charge_min_flat_w] = ComputeMinimalChargingRequirements(obj)

c_vec = 1:obj.C;

% Sum across M
E_start_full_vehicle_pre_1 = sum(obj.FullVehicleInitialPos,1);
% Sum across N
E_start_full_vehicle_pre_2 = squeeze(sum(E_start_full_vehicle_pre_1,2));

E_start_full_vehicle = c_vec(:).'*E_start_full_vehicle_pre_2(:);

% Sum across N
E_start_empty_vehicle_pre = sum(obj.EmptyVehicleInitialPos,1);

E_start_empty_vehicle = c_vec(:).'*E_start_empty_vehicle_pre(:);

E_start = E_start_full_vehicle + E_start_empty_vehicle;

% Recall that MinEndCharge is exclusive
min_end_charge_true = obj.MinEndCharge + 1;

E_end_min = min_end_charge_true*obj.eamod_spec.n_vehicle;

G_charge = digraph(obj.ChargeToTraverse);

ChargeToTraverse_shortest_path = zeros(obj.N,obj.N);

for i = 1:obj.N
    for j = 1:obj.N
        % Minimal charge to get from i to j
        [~,min_charge_ij] =  shortestpath(G_charge,i,j);
        
        ChargeToTraverse_shortest_path(i,j) = min_charge_ij;        
    end    
end

E_trip_min_cell = cell(obj.M,1);

for m = 1:obj.M
    sink = obj.Sinks(m);
    
    n_sources_for_sink = numel(obj.Sources{m});
    
    E_trip_min_source_sink_array = zeros(n_sources_for_sink,1);   
    
    for i_sources_for_sink = 1:n_sources_for_sink
        source = obj.Sources{m}(i_sources_for_sink);
        flow = obj.Flows{m}(i_sources_for_sink);
        
        charge_source_sink = ChargeToTraverse_shortest_path(source,sink);
        E_trip_min_source_sink = charge_source_sink*flow;                
        
        E_trip_min_source_sink_array(i_sources_for_sink) = E_trip_min_source_sink;
    end   
    
    E_trip_min_cell{m} = E_trip_min_source_sink_array;    
end

E_trip_min_per_sink = cellfun(@(x) sum(x),E_trip_min_cell);

E_trip_min = sum(E_trip_min_per_sink);

E_charge_min = E_end_min + E_trip_min - E_start;

E_charge_min_j = E_charge_min*obj.charge_unit_j;

P_charge_min_flat_w = E_charge_min_j/(obj.Thor*obj.time_step_s*obj.NumChargers);
end















