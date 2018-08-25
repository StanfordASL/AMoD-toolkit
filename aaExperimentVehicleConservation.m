function aaExperimentVehicleConservation
data_path = 'Test/dfw_roadgraph_kmeans_tv_federico_5cl_windsun_12h_v3_feas.mat';

scenario = LoadScenario(data_path);  

spec = EAMoDspec.CreateFromScenario(scenario);

eamod_problem = EAMoDproblem(spec);

eamod_problem.Solve();

eamod_problem.PlotRoadGraph();
eamod_problem.PlotVehicleState();
eamod_problem.PlotDeparturesAndArrivals();
end