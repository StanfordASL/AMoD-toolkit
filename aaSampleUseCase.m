% aaSampleUseCase Sample use case of AMoD-toolkit that compares the standard and the real-time formulations in terms of objective value and solver time

clear variables;

%% Prepare specification
% Load demo EAMoDspec
data = load('EAMoDspecDemo_25n_8t_30c.mat');
spec = data.spec;

% Add random charger electricity prices in [35,45] USD per MWh
% Seed for repeatability
rng('default');
charger_electricity_price_usd_per_j = (35 + 10*rand(spec.n_charger,spec.n_time_step))/(1e6*3600);
spec.charger_electricity_price_usd_per_j = charger_electricity_price_usd_per_j;

%% Explore solution to EAMoDproblem in the traditional formulation

% Instantiate EAMoDproblem based on spec
eamod_problem = EAMoDproblem(spec);

% Solve optimization problem
[objective_value,solver_time_s] = eamod_problem.Solve();

% Evaluate the different cost components in the optimal solution
[amod_cost_usd,pax_cost_usd,reb_cost_usd] = eamod_problem.EvaluateAMoDcost();
electricity_cost_usd = eamod_problem.EvaluateElectricityCost();

%% Plot results
eamod_problem.PlotRoadGraph();
eamod_problem.PlotVehicleState();
eamod_problem.PlotDeparturesAndArrivals();

%% Explore solution to EAMoDproblem in the real-time formulation

% Instantiate EAMoDproblemRT based on spec
eamod_problem_rt = EAMoDproblemRT(spec);

[objective_value_rt,solver_time_s_rt] = eamod_problem_rt.Solve();

% Evaluate the different cost components in the optimal solution
[amod_cost_usd_rt,pax_cost_usd_rt,reb_cost_usd_rt] = eamod_problem_rt.EvaluateAMoDcost();
electricity_cost_usd_rt = eamod_problem_rt.EvaluateElectricityCost();

% Explore relative difference in amod_cost and solver_time_s for traditional and real-time formulations
amod_cost_rel_difference = (amod_cost_usd_rt - amod_cost_usd)/amod_cost_usd
solver_time_s_rel_difference = (solver_time_s_rt - solver_time_s)/solver_time_s

%% Plot results
eamod_problem_rt.PlotVehicleState();
eamod_problem_rt.PlotDeparturesAndArrivals();