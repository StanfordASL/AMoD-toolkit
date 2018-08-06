function aaExperimentVehicleConservation
data_path = 'Test/dfw_roadgraph_kmeans_tv_federico_5cl_windsun_12h_v3_feas.mat';

scenario = LoadScenario(data_path);  

spec = EAMoDspec.CreateFromScenario(scenario);
eamod_problem = EAMoDproblemBase(spec);

eamod_problem.Solve();

n_start_vehicles = eamod_problem.ComputeNumberOfVehiclesAtStart();
n_end_vehicles = eamod_problem.ComputeNumberOfVehiclesAtEnd();

verifyEqual(test_case,n_start_vehicles,n_end_vehicles,'RelTol',test_case.TestData.rel_tol_equality);

[ChargingVehicleHist,DischargingVehicleHist,PaxVehicleHist,...
    RebVehicleHist,IdleVehicleHist,AllVehicleHist] ...
    = eamod_problem.GetVehicleStateHistograms();

verifyEqual(test_case,AllVehicleHist,n_start_vehicles*ones(size(AllVehicleHist)),'RelTol',test_case.TestData.rel_tol_equality);


end