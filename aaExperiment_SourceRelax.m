function aaExperiment_SourceRelax

data_path = 'Test/dfw_roadgraph_kmeans_tv_federico_5cl_windsun_12h_v3_feas';
scenario = GetScenario(data_path);

scneario_relax = scenario;
scneario_relax.Flags.sourcerelaxflag = true;

% Check if result from TVPowerBalancedFlow_withpower_sinkbundle is feasible in EAMoDproblem
[cplex_out_relax,fval_relax,~,~,~,~,~,~,lp_matrices_relax] = ...
    TVPowerBalancedFlow_withpower_sinkbundle(scneario_relax.Thor,scneario_relax.RoadNetwork,scneario_relax.PowerNetwork,scneario_relax.InitialConditions,scneario_relax.RebWeight,scneario_relax.Passengers,scneario_relax.Flags);

spec = EAMoDspec.CreateFromScenario(scenario);

eamod_problem_relax = EAMoDproblemBase(spec);
eamod_problem_relax.yalmip_settings = sdpsettings('solver','linprog');

eamod_problem_relax.sourcerelaxflag = scneario_relax.Flags.sourcerelaxflag;
eamod_problem_relax.SourceRelaxCost = lp_matrices_relax.SourceRelaxCost;

indexer = GetIndexer(scneario_relax.Thor,scneario_relax.RoadNetwork,scneario_relax.PowerNetwork,scneario_relax.InitialConditions,scneario_relax.RebWeight,scneario_relax.Passengers,scneario_relax.Flags);
state_range = GetStateRange(eamod_problem_relax,indexer);
decision_vector_val_ref = cplex_out_relax(state_range);

eamod_problem_relax.decision_variables = eamod_problem_relax.DefineDecisionVariables();
assign(eamod_problem_relax.decision_variables.decision_vector,decision_vector_val_ref)

constraint_array = eamod_problem_relax.GetConstraintArray();
check(constraint_array)

[cplex_out,fval,~,~,~,~,~,~,lp_matrices] = ...
    TVPowerBalancedFlow_withpower_sinkbundle(scenario.Thor,scenario.RoadNetwork,scenario.PowerNetwork,scenario.InitialConditions,scenario.RebWeight,scenario.Passengers,scenario.Flags);



% Compare objective values
eamod_problem = EAMoDproblemBase(spec);
eamod_problem.yalmip_settings = sdpsettings('solver','linprog');

eamod_problem_relax = EAMoDproblemBase(spec);
eamod_problem_relax.yalmip_settings = sdpsettings('solver','linprog');

eamod_problem_relax.sourcerelaxflag = scneario_relax.Flags.sourcerelaxflag;
eamod_problem_relax.SourceRelaxCost = lp_matrices_relax.SourceRelaxCost;

[objective_value,solver_time,diagnostics] = eamod_problem.Solve();
[objective_value_relax,solver_time_relax,diagnostics_relax] = eamod_problem_relax.Solve();



end
