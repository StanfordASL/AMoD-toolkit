function tests = VehicleConservationTest
% VehicleConservationTest verifies that vehicles are conserved across time

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

eamod_problem_feas = EAMoDproblemBase(spec_feas); 

scenario_infeas = LoadScenario('dfw_roadgraph_kmeans_tv_federico_5cl_windsun_12h_v3_infeas');
spec_infeas = EAMoDspec.CreateFromScenario(scenario_infeas); 

eamod_problem_infeas = EAMoDproblemBase(spec_infeas); 

eamod_problem_infeas.sourcerelaxflag = true;
eamod_problem_infeas.SourceRelaxCost = 1e8;

HelperVerifyVehicleConservation_1(test_case,eamod_problem_feas)
HelperVerifyVehicleConservation_1(test_case,eamod_problem_infeas)
end

function HelperVerifyVehicleConservation_1(test_case,eamod_problem)
eamod_problem.Solve();
HelperVerifyVehicleConservation_2(test_case,eamod_problem);

eamod_problem.use_real_time_formulation = true;
eamod_problem.Solve();
HelperVerifyVehicleConservation_2(test_case,eamod_problem);
end


function HelperVerifyVehicleConservation_2(test_case,eamod_problem)
n_start_vehicles = eamod_problem.ComputeNumberOfVehiclesAtStart();
n_end_vehicles = eamod_problem.ComputeNumberOfVehiclesAtEnd();

verifyEqual(test_case,n_start_vehicles,n_end_vehicles,'RelTol',test_case.TestData.rel_tol_equality);

[~,~,~,~,~,AllVehicleHist] = eamod_problem.GetVehicleStateHistograms();

verifyEqual(test_case,AllVehicleHist,n_start_vehicles*ones(size(AllVehicleHist)),'RelTol',test_case.TestData.rel_tol_equality);
end










