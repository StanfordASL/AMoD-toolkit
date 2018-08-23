function aaExperiment_SourceRelax_Infeasible
% Here we create a scenario that is infeasible for non-relaxed problem

data_path = 'Test/dfw_roadgraph_kmeans_tv_federico_5cl_windsun_12h_v3_feas';
scenario = GetScenario(data_path);

% Reduce number of vehicles
factor = 1;
scenario.InitialConditions.EmptyVehicleInitialPos = factor*scenario.InitialConditions.EmptyVehicleInitialPos;

[cplex_out,fval,~,~,~,~,~,~,lp_matrices] = ...
    TVPowerBalancedFlow_withpower_sinkbundle(scenario.Thor,scenario.RoadNetwork,scenario.PowerNetwork,scenario.InitialConditions,scenario.RebWeight,scenario.Passengers,scenario.Flags);


scenario.Flags.sourcerelaxflag = true;

[cplex_out,fval,~,~,~,~,~,~,lp_matrices] = ...
    TVPowerBalancedFlow_withpower_sinkbundle(scenario.Thor,scenario.RoadNetwork,scenario.PowerNetwork,scenario.InitialConditions,scenario.RebWeight,scenario.Passengers,scenario.Flags);

spec = EAMoDspec.CreateFromScenario(scenario);

eamod_problem = EAMoDproblemBase(spec);
eamod_problem.yalmip_settings = sdpsettings('solver','linprog');
eamod_problem.sourcerelaxflag = scenario.Flags.sourcerelaxflag;
eamod_problem.SourceRelaxCost = lp_matrices.SourceRelaxCost;

[objective_value,solver_time,diagnostics] = eamod_problem.Solve();

indexer = GetIndexer(scenario.Thor,scenario.RoadNetwork,scenario.PowerNetwork,scenario.InitialConditions,scenario.RebWeight,scenario.Passengers,scenario.Flags);
state_range = GetStateRange(eamod_problem,indexer);
decision_vector_val_ref = cplex_out(state_range);

decision_vector_val = eamod_problem.EvaluateDecisionVector();


assign(eamod_problem.decision_variables.decision_vector,decision_vector_val_ref)

constraint_array = eamod_problem.GetConstraintArray();
check(constraint_array)



end