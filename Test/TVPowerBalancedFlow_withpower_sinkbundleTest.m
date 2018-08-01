% Here we verify that TVPowerBalancedFlow_withpower_sinkbundle matches 
% TVPowerBalancedFlow_withpower_sinkbundle_legacy
function tests = TVPowerBalancedFlow_withpower_sinkbundleTest
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

mosek_path = '/home/ealvaro/ProgramFiles/mosek/8/toolbox/r2014a';
addpath(genpath(mosek_path));

test_case.TestData.rel_tol_equality = 1e-6;

test_case.TestData.data_path_cell = {'dfw_roadgraph_kmeans_tv_federico_5cl_windsun_12h_v3_feas'};
end

function TestOptimizationResultsMatch(test_case)

n_data_path = numel(test_case.TestData.data_path_cell);

for i_data_path = 1:n_data_path
    data_path = test_case.TestData.data_path_cell{i_data_path};

    scenario = LoadScenario(data_path);

    % Relax because otherwise it is infrasible
    scenario.Flags.sourcerelaxflag = true;

    [cplex_out_empty_ref,fval_empty_ref,exitflag_empty_ref,output_empty_ref,lambdas_empty_ref,dual_prices_ix_empty_ref,dual_charger_prices_ix_empty_ref] = TVPowerBalancedFlow_withpower_sinkbundle(scenario.Thor,scenario.RoadNetworkEmpty,scenario.PowerNetwork,scenario.InitialConditions,scenario.RebWeight,scenario.PassengersEmpty,scenario.Flags);
    [cplex_out_ref,fval_ref,exitflag_ref,output_ref,lambdas_ref,dual_prices_ix_ref,dual_charger_prices_ix_ref]=TVPowerBalancedFlow_withpower_sinkbundle(scenario.Thor,scenario.RoadNetwork,scenario.PowerNetwork,scenario.InitialConditions,scenario.RebWeight,scenario.Passengers,scenario.Flags);


    [cplex_out_empty,fval_empty,exitflag_empty,output_empty,lambdas_empty,dual_prices_ix_empty,dual_charger_prices_ix_empty] = TVPowerBalancedFlow_withpower_sinkbundle_legacy(scenario.Thor,scenario.RoadNetworkEmpty,scenario.PowerNetwork,scenario.InitialConditions,scenario.RebWeight,scenario.PassengersEmpty,scenario.Flags);
    [cplex_out,fval,exitflag,output,lambdas,dual_prices_ix,dual_charger_prices_ix] = TVPowerBalancedFlow_withpower_sinkbundle_legacy(scenario.Thor,scenario.RoadNetwork,scenario.PowerNetwork,scenario.InitialConditions,scenario.RebWeight,scenario.Passengers,scenario.Flags);

    verifyEqual(test_case,cplex_out_empty,cplex_out_empty_ref,'RelTol',test_case.TestData.rel_tol_equality)
    verifyEqual(test_case,fval_empty,fval_empty_ref,'RelTol',test_case.TestData.rel_tol_equality)
    verifyEqual(test_case,lambdas_empty,lambdas_empty_ref,'RelTol',test_case.TestData.rel_tol_equality)
    verifyEqual(test_case,dual_prices_ix_empty,dual_prices_ix_empty_ref,'RelTol',test_case.TestData.rel_tol_equality)
    verifyEqual(test_case,dual_charger_prices_ix_empty,dual_charger_prices_ix_empty_ref,'RelTol',test_case.TestData.rel_tol_equality)

    verifyEqual(test_case,cplex_out,cplex_out_ref,'RelTol',test_case.TestData.rel_tol_equality)
    verifyEqual(test_case,fval,fval_ref,'RelTol',test_case.TestData.rel_tol_equality)
    verifyEqual(test_case,lambdas,lambdas_ref,'RelTol',test_case.TestData.rel_tol_equality)
    verifyEqual(test_case,dual_prices_ix,dual_prices_ix_ref,'RelTol',test_case.TestData.rel_tol_equality)
    verifyEqual(test_case,dual_charger_prices_ix,dual_charger_prices_ix_ref,'RelTol',test_case.TestData.rel_tol_equality)
end
end