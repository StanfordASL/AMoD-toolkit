% aaSampleUseCase Sample use case of AMoD-toolkit that compares the standard and the real-time formulations in terms of objective value and solver time

clear variables;

%% Load demo EAMoDspec
data = load('EAMoDspecDemo.mat');
spec = data.spec;

%% Explore solution to EAMoDproblem in the real-time formulation
% This should take less than a minute 

% Instantiate EAMoDproblemRT based on spec
% This displays a warning saying that "Residual road capacity is negative." In the real-time
% formulation, the passenger-carrying vehicles follow fixed routes independent of congestion.
% This warning tells us that there are some road links where the flow from pre-routed passenger-
% carrying exceeds the link's capacity. In these cases, the code sets the link's residual
% capacity to zero so that rebalancing vehicles avoid the links which are already congested.
eamod_problem_rt = EAMoDproblemRT(spec);

[objective_value_rt,solver_time_s_rt] = eamod_problem_rt.Solve();

% Evaluate the different cost components in the optimal solution
[amod_cost_usd_rt,pax_cost_usd_rt,reb_cost_usd_rt] = eamod_problem_rt.EvaluateAMoDcost();
electricity_cost_usd_rt = eamod_problem_rt.EvaluateElectricityCost();

%% Plot results
eamod_problem_rt.PlotRoadGraph();
fig_handle_vehicle_state_rt = eamod_problem_rt.PlotVehicleState();
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
fig_handle_vehicle_state = eamod_problem.PlotVehicleState();
eamod_problem.PlotDeparturesAndArrivals();

%% Explore relative difference in amod_cost and solver_time_s for traditional and real-time formulations
amod_cost_rel_difference = (amod_cost_usd_rt - amod_cost_usd)/amod_cost_usd
solver_time_s_rel_difference = (solver_time_s_rt - solver_time_s)/solver_time_s

%% Save vehicle state plots
params_plot = GetParamsPlot();

ConfigurePlot(params_plot,fig_handle_vehicle_state_rt);
ConfigurePlot(params_plot,fig_handle_vehicle_state);

print(fig_handle_vehicle_state_rt,'state_plot_rt','-dsvg')
print(fig_handle_vehicle_state,'state_plot','-dsvg')
