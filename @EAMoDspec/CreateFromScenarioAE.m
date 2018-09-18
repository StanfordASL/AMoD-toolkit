function obj = CreateFromScenarioAE(scenario)
% CreateFromScenarioAE Instantiates EAMoDspec based on a legacy scenario (AE version)

% Get rid of ints because working with them in Matlab is a hassle
data = ConvertNumbersInStructToDouble(scenario);

obj = EAMoDspec();

obj.n_time_step = data.n_time_step;
obj.n_charge_step = data.C;

% Road network
obj.road_adjacency_list = data.adjacencies;
obj.road_capacity_matrix = data.cap_matrix;
obj.road_travel_time_matrix = data.time_matrix;
obj.road_travel_distance_matrix_m = data.dist_matrix_m;
obj.road_charge_to_traverse_matrix = data.charge_req_matrix;

% Charger data
obj.charger_list = data.charger_list;
obj.charger_speed = data.charging_speeds;
obj.charger_time = data.charging_times;
obj.charger_capacity = data.charger_cap;

obj.charger_electricity_price_usd_per_j = zeros(obj.n_charger,obj.n_time_step);

for i_charger = 1:obj.n_charger
    substation_name = data.charger_substation_name_list(i_charger);
    
    i_substation = find(strcmp(substation_name,data.substation_name_list));    
    
    obj.charger_electricity_price_usd_per_j(i_charger,:) = data.substation_price_array_usd_per_j{i_substation};    
end

% Trip demand
CompactSinks = unique(data.sinks);
CompactSinks = CompactSinks(:);

% Do flow bundling
CompactSources = cell(numel(CompactSinks),1);
CompactStartTimes = cell(numel(CompactSinks),1);
CompactFlows = cell(numel(CompactSinks),1);

for i=1:numel(CompactSinks)
    CompactSources{i} = data.sources(data.sinks == CompactSinks(i));
    CompactStartTimes{i} = data.start_times(data.sinks == CompactSinks(i));
    CompactFlows{i} = data.flows(data.sinks == CompactSinks(i));
end

obj.passenger_sink_list = CompactSinks;
obj.passenger_source_list_cell = CompactSources;
obj.passenger_start_time_list_cell = CompactStartTimes;
obj.passenger_flow_list_cell = CompactFlows;

% Initial conditions
init_distr = zeros(obj.n_road_node,1);
for i=1:obj.n_road_node
    init_distr(i) = sum(data.flows(data.sources == i & data.start_times == 1));
end
init_distr = init_distr / sum(init_distr);

num_vehicles = data.n_vehicle;

obj.initial_state_empty_vehicles = zeros(obj.n_road_node, obj.n_charge_step);
% Recall that MinEndCharge is exclusive
obj.initial_state_empty_vehicles(:,data.min_end_charge + 1) = num_vehicles*init_distr;

obj.initial_state_full_vehicles = zeros(obj.n_passenger_flow,obj.n_road_node,obj.n_charge_step);

% Final conditions
obj.final_min_charge  = data.min_end_charge;

% Economic parameters
obj.value_of_time_usd_per_time_step = data.value_of_time_usd_per_step;
obj.vehicle_cost_usd_per_m = data.vehicle_cost_usd_per_m;

% Others
obj.time_step_s = data.time_step_s;
obj.charge_step_j = data.charge_unit_j;

% For now, inhibit v2g
obj.v2g_efficiency = 0;

obj.start_date_time = datetime(data.start_date);

end

function res_struct = ConvertNumbersInStructToDouble(data_struct)
for field_c = fields(data_struct).'
    field = field_c{:};
    
    value = data_struct.(field);
    
    switch class(value)
        case 'cell'
            res = ConvertNumbersInCellToDouble(value);            
        case 'struct'
            res = ConvertNumbersInStructToDouble(value);            
        otherwise
            if isnumeric(value)
                res = double(value);
            else
                res = value;
            end  
    end
    
    res_struct.(field) = res;    
end
end

function res_cell = ConvertNumbersInCellToDouble(data_cell)

n_data_cell = numel(data_cell);

res_cell = cell(size(data_cell));

for i_data_cell = 1:n_data_cell
    
    value = data_cell{i_data_cell};
    
    switch class(value)
        case 'cell'
            res = ConvertNumbersInCellToDouble(value);            
        otherwise
            if isnumeric(value)
                res = double(value);
            else
                res = value;
            end
    end
    
    res_cell{i_data_cell} = res;
end

end