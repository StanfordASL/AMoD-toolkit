function aaExperimentRoadCap
data_path = 'Test/dfw_roadgraph_kmeans_tv_federico_5cl_windsun_12h_v3_feas.mat';

scenario = LoadScenario(data_path);  

spec =  EAMoDspec.CreateFromScenario(scenario);
eamod_problem = EAMoDproblem(spec);

TVRoadCap_scen = permute(scenario.RoadNetwork.TVRoadCap,[2,3,1]);

TVRoadCap = eamod_problem.TVRoadCap;

TVRoadCap_delta = TVRoadCap - TVRoadCap_scen;

end