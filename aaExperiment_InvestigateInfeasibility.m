scenario = LoadScenario('dfw_25cl_4h_30mstride_demo');

spec = EAMoDspec.CreateFromScenarioFR(scenario);
spec = spec.RemoveTripsEndingTooLate();

spec.road_capacity_matrix = 12.5*3/4*spec.road_capacity_matrix;

initial_charge = 25;
initial_state_empty_vehicles = spec.initial_state_empty_vehicles;
spec.initial_state_empty_vehicles(:,initial_charge) = sum(initial_state_empty_vehicles,2);
% % Keep in mind that it is exclusive
% spec.final_min_charge = 5;

eamod_problem_rt = EAMoDproblemRT(spec);
eamod_problem_rt.Solve();

eamod_problem = EAMoDproblem(spec);
eamod_problem.Solve();


% Relax and evaluate relaxation
eamod_problem_rt.source_relax_cost = 1e8;
eamod_problem_rt.source_relax_flag = true;

eamod_problem_rt.Solve();


% Evaluate relaxation

state_vec = eamod_problem_rt.EvaluateStateVector();

relax_vec = state_vec(eamod_problem_rt.FindSourceRelaxks(1,1):end);

reference_source_sink_time = zeros(numel(relax_vec),3);
i_x = 0;

for k = 1:spec.n_passenger_flow
   sink = spec.passenger_sink_list(k);
    
   source_list = spec.passenger_source_list_cell{k};
   
   for i_source = 1:numel(source_list)
       source = source_list(i_source);
       
       i_x = i_x + 1;
       
       reference_source_sink_time(i_x,1) = source;
       reference_source_sink_time(i_x,2) = sink;
       reference_source_sink_time(i_x,3) = spec.passenger_start_time_list_cell{k}(i_source);       
   end    
end

is_relaxed = relax_vec > 0;
relaxed_vec = relax_vec(is_relaxed);
relaxed_source_sink_time = reference_source_sink_time(is_relaxed,:); 

insuficient_source = unique(relaxed_source_sink_time(:,1));




