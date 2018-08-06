function tests = VehicleConservationTest
% This enables us to run the test by calling the functinwithout calling it from runtests
call_stack = dbstack;

% Call stack has only one element if function was called directly
if ~any(contains({call_stack.name},'runtests'))
    this_file_name = mfilename();
    runtests(this_file_name)
end

tests = functiontests(localfunctions);
end

function setupOnce(test_case)
addpath('..')
addpath('HelperFunctions');

test_case.TestData.rel_tol_equality = 1e-6;

test_case.TestData.data_path_cell = {'dfw_roadgraph_kmeans_tv_federico_5cl_windsun_12h_v3_feas'};
end

function TestVehicleConservation(test_case)

n_data_path = numel(test_case.TestData.data_path_cell);

for i_data_path = 1:n_data_path
    data_path = test_case.TestData.data_path_cell{i_data_path};
    
    scenario = LoadScenario(data_path);    
    spec = EAMoDspec.CreateFromScenario(scenario);   
    
    eamod_problem = EAMoDproblemBase(spec);    
    
    HelperVerifyVehicleConservation(test_case,eamod_problem)
end

end

function HelperVerifyVehicleConservation(test_case,eamod_problem)
eamod_problem.Solve();

n_start_vehicles = eamod_problem.ComputeNumberOfVehiclesAtStart();
n_end_vehicles = eamod_problem.ComputeNumberOfVehiclesAtEnd();

verifyEqual(test_case,n_start_vehicles,n_end_vehicles,'RelTol',test_case.TestData.rel_tol_equality);

[~,~,~,~,~,AllVehicleHist] = eamod_problem.GetVehicleStateHistograms();

verifyEqual(test_case,AllVehicleHist,n_start_vehicles*ones(size(AllVehicleHist)),'RelTol',test_case.TestData.rel_tol_equality);

end

