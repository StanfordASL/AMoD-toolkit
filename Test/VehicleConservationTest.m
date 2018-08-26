function tests = VehicleConservationTest
% VehicleConservationTest Verifies that vehicles are conserved across time

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
end

function TestVehicleConservation(test_case)

scenario_feas = LoadScenario('dfw_roadgraph_kmeans_tv_federico_5cl_windsun_12h_v3_feas');
spec_feas = EAMoDspec.CreateFromScenario(scenario_feas); 

eamod_problem_feas = EAMoDproblem(spec_feas); 
eamod_problem_feas_rt = EAMoDproblemRT(spec_feas); 

HelperVerifyVehicleConservation(test_case,eamod_problem_feas)
HelperVerifyVehicleConservation(test_case,eamod_problem_feas_rt)

scenario_infeas = LoadScenario('dfw_roadgraph_kmeans_tv_federico_5cl_windsun_12h_v3_infeas');
spec_infeas = EAMoDspec.CreateFromScenario(scenario_infeas); 

% NOTE: we do not test vehicle conservation for source-relaxed real-time formulation. 
% It does not conserve vehicles in the sense of this test.
eamod_problem_infeas = EAMoDproblem(spec_infeas); 

eamod_problem_infeas.source_relax_flag = true;
eamod_problem_infeas.source_relax_cost = 1e8;
HelperVerifyVehicleConservation(test_case,eamod_problem_infeas)
end

function HelperVerifyVehicleConservation(test_case,eamod_problem)
eamod_problem.Solve();

n_start_vehicles = eamod_problem.ComputeNumberOfVehiclesAtStart();
n_end_vehicles = eamod_problem.ComputeNumberOfVehiclesAtEnd();

verifyEqual(test_case,n_start_vehicles,n_end_vehicles,'RelTol',test_case.TestData.rel_tol_equality);

[~,~,~,~,~,AllVehicleHist] = eamod_problem.GetVehicleStateHistograms();

verifyEqual(test_case,AllVehicleHist,n_start_vehicles*ones(size(AllVehicleHist)),'RelTol',test_case.TestData.rel_tol_equality);
end










