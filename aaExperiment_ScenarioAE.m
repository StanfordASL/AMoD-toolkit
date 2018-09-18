function aaExperiment_ScenarioAE
scenario = load('output_santa_ana_plus_10r_25t_30c');
scenario.n_vehicle = 18000;

spec = EAMoDspec.CreateFromScenarioAE(scenario);

%% Explore solution to EAMoDproblem in the real-time formulation
% This should take less than a minute 

% Instantiate EAMoDproblemRT based on spec
eamod_problem_rt = EAMoDproblemRT(spec);

[objective_value_rt,solver_time_s_rt] = eamod_problem_rt.Solve();

% Evaluate the different cost components in the optimal solution
[amod_cost_usd_rt,pax_cost_usd_rt,reb_cost_usd_rt] = eamod_problem_rt.EvaluateAMoDcost();
electricity_cost_usd_rt = eamod_problem_rt.EvaluateElectricityCost();

%% Plot results
eamod_problem_rt.PlotRoadGraph();
eamod_problem_rt.PlotVehicleState();
eamod_problem_rt.PlotDeparturesAndArrivals();

%% Explore solution to EAMoDproblem in the traditional formulation
% This will take 5-10 minutes depending on solver and computer performance.

% Instantiate EAMoDproblem based on spec
eamod_problem = EAMoDproblem(spec);

% Solve optimization problem
[objective_value,solver_time_s] = eamod_problem.Solve();

% Evaluate the different cost components in the optimal solution
[amod_cost_usd,pax_cost_usd,reb_cost_usd] = eamod_problem.EvaluateAMoDcost();
electricity_cost_usd = eamod_problem.EvaluateElectricityCost();

%% Plot results
eamod_problem.PlotVehicleState();
eamod_problem.PlotDeparturesAndArrivals();

%% Explore relative difference in amod_cost and solver_time_s for traditional and real-time formulations
amod_cost_rel_difference = (amod_cost_usd_rt - amod_cost_usd)/amod_cost_usd
solver_time_s_rel_difference = (solver_time_s_rt - solver_time_s)/solver_time_s

end



