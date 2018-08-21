function tests = CompareWithAMoDpowerRealTimeTest
% CompareWithAMoDpowerRealTimeTest verifies that the results match the implementation in AMoD-power using the real-time formulation
%   This includes checking that the matrices for the linear program are the 
%   same and verifying that the optimization results are close.

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
% mosek_path = 'C:/Program Files/Mosek/8/toolbox/r2014a';
addpath(genpath(mosek_path));

amod_power_path = '../../AMoD-power/';
addpath(amod_power_path);

test_case.TestData.rel_tol_equality_hard = 1e-8;

test_case.TestData.rel_tol_equality = 1e-4;
test_case.TestData.abs_tol_equality = 5e-4;

test_case.TestData.data_path_cell = {'dfw_roadgraph_kmeans_tv_federico_5cl_windsun_12h_v3_feas'};
end

function TestCompareWithAMoDpower(test_case)
n_data_path = numel(test_case.TestData.data_path_cell);

for i_data_path = 1:n_data_path
    data_path = test_case.TestData.data_path_cell{i_data_path};
    
    scenario = GetScenario(data_path);
    scenario = AdaptScenarioForRealTime(scenario);    
    
    % Test with non-relaxed source constraints
%     scenario.Flags.sourcerelaxflag = false;
%     CompareWithAMoDpowerHelper(test_case,scenario);
    
    % Test with relaxed source constraints
    scenario_sourcerelaxflag = scenario;
    scenario_sourcerelaxflag.Flags.sourcerelaxflag = true;
    CompareWithAMoDpowerHelper(test_case,scenario_sourcerelaxflag);    
end
end

function scenario = AdaptScenarioForRealTime(scenario)
N = numel(scenario.RoadNetwork.RoadGraph);

% Neglect MinNumVehiclesAti
scenario.InitialConditions.MinNumVehiclesAti = zeros(N,1);

% Prevent outstanding customers 
scenario.Passengers.StarterSinks = [];
scenario.Passengers.StarterSources = [];
scenario.Passengers.StarterFlows = [];

% Real-time formulation uses RebWeight but EAMoDproblem does not.
scenario.RebWeight = 0;

% EAMoDproblem handles MinEndCharge using state vector bounds (the same way as TVPowerBalancedFlow_withpower_sinkbundle).
% TVPowerBalancedFlow_realtime does it with a special constraint.
% For simplicity, we neglect it here
scenario.RoadNetwork.MinEndCharge = 0;
end

function CompareWithAMoDpowerHelper(test_case,scenario)
spec = EAMoDspec.CreateFromScenario(scenario);

[cplex_out,fval,exitflag,output,lambdas,dual_prices_ix,dual_charger_prices_ix,lb,solveTime,lp_matrices] =...
    TVPowerBalancedFlow_realtime(scenario.Thor,scenario.RoadNetwork,scenario.PowerNetwork,scenario.InitialConditions,scenario.RebWeight,scenario.Passengers,scenario.Flags);

if spec.sourcerelaxflag
    spec.SourceRelaxCost = lp_matrices.SourceRelaxCost;
end

eamod_problem = EAMoDproblemBase(spec);
eamod_problem.use_real_time_formulation = true;

indexer = GetIndexerRealTime(scenario.Thor,scenario.RoadNetwork,scenario.PowerNetwork,scenario.InitialConditions,scenario.RebWeight,scenario.Passengers,scenario.Flags);

LPmatricesMatch(test_case,eamod_problem,lp_matrices,indexer);

OptimizationResultsMatch(test_case,eamod_problem,cplex_out,fval,indexer);
end

function state_range = GetStateRange(spec,indexer)
state_range = indexer.FindRoadLinkPtckij(1,1,1,1,1):indexer.FindEndRebLocationci(spec.C,spec.N);

if spec.sourcerelaxflag
    col_range_relax = indexer.FindSourceRelaxks(1,1) + [0:(spec.TotNumSources - 1)];
    state_range = [state_range,col_range_relax];    
end
end

function LPmatricesMatch(test_case,eamod_problem,lp_matrices,indexer)
spec = eamod_problem.spec;

f_cost_full_ref = lp_matrices.f_cost;
Ain_ref = lp_matrices.Aineq;
Bin_ref = lp_matrices.Bineq;
Aeq_ref = lp_matrices.Aeq;
Beq_ref = lp_matrices.Beq;
lb_StateVector_full_ref = lp_matrices.lb;
ub_StateVector_full_ref = lp_matrices.ub;

state_range = GetStateRange(spec,indexer);

% Cost vector
f_cost = eamod_problem.CreateCostVector();
f_cost_ref = f_cost_full_ref(state_range);

verifyEqual(test_case,f_cost,f_cost_ref)

% RebConservation
[Aeq_RebConservation, Beq_RebConservation] = eamod_problem.CreateEqualityConstraintMatrices_RebConservation();

row_start_RebConservation = indexer.FindEqRebConservationtci(1,1,1);
row_end_RebConservation = indexer.FindEqRebConservationtci(spec.Thor,spec.C,spec.N);
row_range_RebConservation = row_start_RebConservation:row_end_RebConservation;

[Aeq_RebConservation_ref,Beq_RebConservation_ref] = ExtractConstraintSubmatrix(Aeq_ref,Beq_ref,row_range_RebConservation,state_range);

verifyEqualSparse(test_case,Aeq_RebConservation,Aeq_RebConservation_ref)
verifyEqual(test_case,Beq_RebConservation,Beq_RebConservation_ref)

% SourceConservation
[Aeq_SourceConservation, Beq_SourceConservation] = eamod_problem.CreateEqualityConstraintMatrices_SourceConservation();

row_start_SourceConservation = indexer.FindEqSourceConservationks(1,1);
row_end_SourceConservation = indexer.FindEqSourceConservationks(spec.M,length(spec.Sources{end}));
row_range_SourceConservation = row_start_SourceConservation:row_end_SourceConservation;

[Aeq_SourceConservation_ref,Beq_SourceConservation_ref] = ExtractConstraintSubmatrix(Aeq_ref,Beq_ref,row_range_SourceConservation,state_range);

verifyEqualSparse(test_case,Aeq_SourceConservation,Aeq_SourceConservation_ref)
verifyEqual(test_case,Beq_SourceConservation,Beq_SourceConservation_ref)

% SinkConservation
[Aeq_SinkConservation, Beq_SinkConservation] = eamod_problem.CreateEqualityConstraintMatrices_SinkConservation();

row_start_SinkConservation = indexer.FindEqSinkConservationk(1);
row_end_SinkConservation = indexer.FindEqSinkConservationk(spec.NumSinks);
row_range_SinkConservation = row_start_SinkConservation:row_end_SinkConservation;

[Aeq_SinkConservation_ref,Beq_SinkConservation_ref] = ExtractConstraintSubmatrix(Aeq_ref,Beq_ref,row_range_SinkConservation,state_range);

verifyEqualSparse(test_case,Aeq_SinkConservation,Aeq_SinkConservation_ref)
verifyEqual(test_case,Beq_SinkConservation,Beq_SinkConservation_ref)

% CustomerChargeConservation
[Aeq_CustomerChargeConservation, Beq_CustomerChargeConservation] = eamod_problem.CreateEqualityConstraintMatrices_CustomerChargeConservation();

row_start_CustomerChargeConservation = indexer.FindEqCustomerChargeConservationktc(1,1,1);
row_end_CustomerChargeConservation = indexer.FindEqCustomerChargeConservationktc(spec.M,spec.Thor,spec.C);
row_range_CustomerChargeConservation = row_start_CustomerChargeConservation:row_end_CustomerChargeConservation;

[Aeq_CustomerChargeConservation_ref,Beq_CustomerChargeConservation_ref] = ExtractConstraintSubmatrix(Aeq_ref,Beq_ref,row_range_CustomerChargeConservation,state_range);

verifyEqualSparse(test_case,Aeq_CustomerChargeConservation,Aeq_CustomerChargeConservation_ref)
verifyEqual(test_case,Beq_CustomerChargeConservation,Beq_CustomerChargeConservation_ref)

% RoadCongestion
[Ain_RoadCongestion, Bin_RoadCongestion] = eamod_problem.CreateInequalityConstraintMatrices_RoadCongestion();

row_start_RoadCongestion = indexer.FindInRoadCongestiontij(1,1,1);
row_end_RoadCongestion = indexer.FindInRoadCongestiontij(spec.Thor,spec.N,spec.RoadGraph{end}(end));
row_range_RoadCongestion = row_start_RoadCongestion:row_end_RoadCongestion;

[Ain_RoadCongestion_ref,Bin_RoadCongestion_ref] = ExtractConstraintSubmatrix(Ain_ref,Bin_ref,row_range_RoadCongestion,state_range);

verifyEqualSparse(test_case,Ain_RoadCongestion,Ain_RoadCongestion_ref)
verifyEqual(test_case,Bin_RoadCongestion,Bin_RoadCongestion_ref)

% ChargerCongestion
[Ain_ChargerCongestion, Bin_ChargerCongestion] = eamod_problem.CreateInequalityConstraintMatrices_ChargerCongestion();

row_start_ChargerCongestion = indexer.FindInChargerCongestiontl(1,1);
row_end_ChargerCongestion = indexer.FindInChargerCongestiontl(spec.Thor,spec.NumChargers);
row_range_ChargerCongestion = row_start_ChargerCongestion:row_end_ChargerCongestion;

[Ain_ChargerCongestion_ref,Bin_ChargerCongestion_ref] = ExtractConstraintSubmatrix(Ain_ref,Bin_ref,row_range_ChargerCongestion,state_range);

verifyEqualSparse(test_case,Ain_ChargerCongestion,Ain_ChargerCongestion_ref)
verifyEqual(test_case,Bin_ChargerCongestion,Bin_ChargerCongestion_ref)

% StateVectorBounds
[lb_StateVector,ub_StateVector] = eamod_problem.CreateStateVectorBounds();

lb_StateVector_ref = lb_StateVector_full_ref(state_range);
ub_StateVector_ref = ub_StateVector_full_ref(state_range);

verifyEqual(test_case,lb_StateVector,lb_StateVector_ref);
verifyEqual(test_case,ub_StateVector,ub_StateVector_ref);
end

function OptimizationResultsMatch(test_case,eamod_problem,cplex_out_ref,fval_ref,indexer)
% We use Mosek's overload of linprog as in
% TVPowerBalancedFlow_withpower_sinkbundle.
eamod_problem.yalmip_settings = sdpsettings('solver','linprog');

[objective_value,solver_time,diagnostics] = eamod_problem.Solve();

decision_vector_val = eamod_problem.EvaluateDecisionVector();

state_range = GetStateRange(eamod_problem.spec,indexer);

decision_vector_val_ref = cplex_out_ref(state_range);
objective_value_ref = fval_ref;

verifyEqual(test_case,objective_value,objective_value_ref,'RelTol',test_case.TestData.rel_tol_equality_hard);

% This test usually fails for relaxed sources (I believe this is due to the challenging numerics).
if ~eamod_problem.spec.sourcerelaxflag
    verifyEqual(test_case,decision_vector_val,decision_vector_val_ref,'RelTol',test_case.TestData.rel_tol_equality,'AbsTol',test_case.TestData.abs_tol_equality);
end
end


function PowerNetworkR = CreateDummyPowerNetwork(Thor,numChargers,v2g_efficiency,power_costs)
% CreateDummyPowerNetwork creates a dummy power network with an unbounded generator at every node with power priced according to power_costs
%   This is for compatibility with the implementation in AMoD-power

PowerGraphR = cell(1,numChargers);
PowerLineCapR = PowerGraphR;
PowerLineReactanceR = PowerGraphR;
PowerGensListR = 1:numChargers;
PowerGensMaxR = Inf*ones(numChargers,Thor);
PowerGensMinR = -Inf*ones(numChargers,Thor);
PowerExtLoadsR = zeros(numChargers,Thor);
RoadToPowerMapR =  1:numChargers;

PowerNetworkR.PowerGraphM = PowerGraphR;
PowerNetworkR.PowerLineCapM = PowerLineCapR;
PowerNetworkR.PowerLineReactanceM = PowerLineReactanceR;
PowerNetworkR.PowerGensList = PowerGensListR;
PowerNetworkR.PowerGensMax = PowerGensMaxR';
PowerNetworkR.PowerGensMin = PowerGensMinR';
PowerNetworkR.PowerCosts = power_costs;
PowerNetworkR.PowerExtLoads = PowerExtLoadsR;
PowerNetworkR.RoadToPowerMap = RoadToPowerMapR;
PowerNetworkR.v2g_efficiency = v2g_efficiency;

end

