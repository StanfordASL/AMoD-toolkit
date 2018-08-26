function aaUseCaseForOptimization

data = load('EAMoDspecDemo.mat');
spec = data.spec;

% Seed for repeatability
rng('default');
% Add random charger electricity prices in [35,45] USD per MWh
charger_electricity_price_usd_per_j = (35 + 10*rand(spec.n_charger,spec.n_time_step))/(1e6*3600);
spec.charger_electricity_price_usd_per_j = charger_electricity_price_usd_per_j;

% Create and solve problem
eamod_problem = EAMoDproblem(spec);

[objective_value,solver_time] = eamod_problem.Solve();

[amod_cost_usd,pax_cost_usd,reb_cost_usd] = eamod_problem.EvaluateAMoDcost();
electricity_cost_usd = eamod_problem.EvaluateElectricityCost();

% Compare with real-time formulation
eamod_problem.use_real_time_formulation = true;

[objective_value_rt,solver_time_rt] = eamod_problem.Solve();

[amod_cost_usd_rt,pax_cost_usd_rt,reb_cost_usd_rt] = eamod_problem.EvaluateAMoDcost();

electricity_cost_usd_rt = eamod_problem.EvaluateElectricityCost();

amod_cost_rel_difference = (amod_cost_usd_rt - amod_cost_usd)/amod_cost_usd
solver_time_rel_difference = (solver_time_rt - solver_time)/solver_time

end


