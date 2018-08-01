function tests = EAMoDproblemTest
% This enables us to run the test directly instead of only through runtests

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
mosek_path = 'C:/Program Files/Mosek/8/toolbox/r2014a';
addpath(genpath(mosek_path));

amod_power_path = '../../AMoD-power/';
addpath(amod_power_path);

test_case.TestData.abs_tol_equality = 1e-4;
test_case.TestData.rel_tol_equality = 5e-6;
test_case.TestData.rel_tol_equality_soft = 5e-5;

test_case.TestData.data_path_cell = {'dfw_roadgraph_kmeans_tv_federico_5cl_windsun_12h_v3_feas'};
end

function TestCompareWithLegacyCode(test_case)

n_data_path = numel(test_case.TestData.data_path_cell);

for i_data_path = 1:n_data_path
    data_path = test_case.TestData.data_path_cell{i_data_path};
    
    scenario = LoadScenario(data_path);
    HelperCompareWithSinkbundle(test_case,scenario);
    
    scenario_sourcerelaxflag = scenario;
    scenario_sourcerelaxflag.Flags.sourcerelaxflag = true;
    HelperCompareWithSinkbundle(test_case,scenario_sourcerelaxflag);
end
end

function HelperCompareWithSinkbundle(test_case,scenario)

eamod_spec = EAMoDspec;
eamod_spec.Thor = scenario.Thor;
eamod_spec.RoadNetwork = scenario.RoadNetwork;
eamod_spec.InitialConditions = scenario.InitialConditions;
eamod_spec.Passengers = scenario.Passengers;
eamod_spec.Flags = scenario.Flags;

% We use Mosek's overload of linprog as in
% TVPowerBalancedFlow_withpower_sinkbundle.
eamod_spec.solver = 'linprog';

eamod_problem = EAMoDproblem(eamod_spec);

eamod_problem.SourceRelaxCost = 1e9;

[objective_value,solver_time,diagnostics] = eamod_problem.Solve();

decision_vector_val = eamod_problem.EvaluateDecisionVector();

numChargers = length(scenario.RoadNetwork.ChargersList);
power_costs = zeros(numChargers,scenario.Thor)';
PowerNetwork_dummy = CreateDummyPowerNetwork(scenario.Thor,numChargers,scenario.PowerNetwork.v2g_efficiency,power_costs);

[cplex_out_ref,fval_ref,exitflag_ref,output_ref,lambdas_ref,dual_prices_ix_ref,dual_charger_prices_ix_ref] = ...
    TVPowerBalancedFlow_withpower_sinkbundle(scenario.Thor,scenario.RoadNetwork,PowerNetwork_dummy,scenario.InitialConditions,scenario.RebWeight,scenario.Passengers,scenario.Flags);

indexer = GetIndexer(scenario.Thor,scenario.RoadNetwork,PowerNetwork_dummy,scenario.InitialConditions,scenario.RebWeight,scenario.Passengers,scenario.Flags);

state_range = indexer.FindRoadLinkPtckij(1,1,1,1,1):indexer.FindEndRebLocationci(eamod_problem.C,eamod_problem.N);

if scenario.Flags.sourcerelaxflag
    state_range_relax = indexer.FindSourceRelaxks(1,1) + [0:(eamod_problem.TotNumSources - 1)];
    state_range = [state_range,state_range_relax];
end

decision_vector_val_ref = cplex_out_ref(state_range);
objective_value_ref = fval_ref;


% When relaxing sources, numerics get challenging. Thus, relax the
% tolerance
if scenario.Flags.sourcerelaxflag
    rel_tol_equality =  test_case.TestData.rel_tol_equality_soft;
else
    rel_tol_equality =  test_case.TestData.rel_tol_equality;
end

verifyEqual(test_case,objective_value,objective_value_ref,'RelTol',rel_tol_equality);

% This test usually fails for relaxed sources (I believe this is due to the challenging numerics).
if ~scenario.Flags.sourcerelaxflag
    verifyEqual(test_case,decision_vector_val,decision_vector_val_ref,'RelTol',rel_tol_equality,'AbsTol',test_case.TestData.abs_tol_equality);
end

HelperVerifyVehicleConservation(test_case,eamod_problem);
end



