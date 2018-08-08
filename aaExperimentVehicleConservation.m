function aaExperimentVehicleConservation
data_path = 'Test/dfw_roadgraph_kmeans_tv_federico_5cl_windsun_12h_v3_feas.mat';

scenario = LoadScenario(data_path);  

spec = EAMoDspec.CreateFromScenario(scenario);
% For PlotTravelTimes
spec.time_step_s = 1800;

eamod_problem = EAMoDproblemBase(spec);

eamod_problem.Solve();

eamod_problem.PlotRoadGraph();
eamod_problem.PlotTravelTimes()
end