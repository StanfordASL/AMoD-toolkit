function tests = CompareWithAMoDpowerTest
% CompareWithAMoDpowerTest Verifies that the results match the implementation in AMoD-power
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

end

function TestCompareWithAMoDpower(test_case)
scenario_feas = LoadScenario('dfw_roadgraph_kmeans_tv_federico_5cl_windsun_12h_v3_feas');

% Test with non-relaxed source constraints
scenario_feas.Flags.sourcerelaxflag = false;
CompareWithAMoDpowerHelper(test_case,scenario_feas,false);
CompareWithAMoDpowerHelper(test_case,scenario_feas,true);

scenario_infeas = LoadScenario('dfw_roadgraph_kmeans_tv_federico_5cl_windsun_12h_v3_infeas');
scenario_infeas.Flags.sourcerelaxflag = true;
CompareWithAMoDpowerHelper(test_case,scenario_infeas,false);
CompareWithAMoDpowerHelper(test_case,scenario_infeas,true);
end

function CompareWithAMoDpowerHelper(test_case,scenario,use_real_time_formulation)

scenario = AddDummyPowerNetworkToScenario(scenario);

spec = EAMoDspec.CreateFromScenario(scenario);

% Seed for repeatability
rng('default');
% Add random charger electricity prices in [35,45] USD per MWh
charger_electricity_price_usd_per_j = (35 + 10*rand(spec.n_charger,spec.n_time_step))/(1e6*3600);
spec.charger_electricity_price_usd_per_j = charger_electricity_price_usd_per_j;

% Legacy code expects PowerCosts to be of size n_time_step x n_charger and be in USD/(BaseMVA*time_step_s)
scenario.PowerNetwork.PowerCosts = charger_electricity_price_usd_per_j.'*(scenario.BaseMVA*1e6*scenario.time_step_s);

% Test non-real time formulation
eamod_problem = EAMoDproblem(spec);

if use_real_time_formulation
    eamod_problem.use_real_time_formulation = true;
    
    [cplex_out,fval,~,~,~,~,~,~,~,lp_matrices] =...
        TVPowerBalancedFlow_realtime(scenario.Thor,scenario.RoadNetwork,scenario.PowerNetwork,scenario.InitialConditions,scenario.RebWeight,scenario.Passengers,scenario.Flags);
else
    [cplex_out,fval,~,~,~,~,~,~,lp_matrices] = ...
        TVPowerBalancedFlow_withpower_sinkbundle(scenario.Thor,scenario.RoadNetwork,scenario.PowerNetwork,scenario.InitialConditions,scenario.RebWeight,scenario.Passengers,scenario.Flags);
end

if scenario.Flags.sourcerelaxflag
    eamod_problem.sourcerelaxflag = true;
    eamod_problem.SourceRelaxCost = lp_matrices.SourceRelaxCost;
end

LPmatricesMatch(test_case,eamod_problem,lp_matrices,scenario);
OptimizationResultsMatch(test_case,eamod_problem,fval);
end

function LPmatricesMatch(test_case,eamod_problem,lp_matrices,scenario)
if eamod_problem.use_real_time_formulation
    indexer = GetIndexerRealTime(scenario.Thor,scenario.RoadNetwork,scenario.PowerNetwork,scenario.InitialConditions,scenario.RebWeight,scenario.Passengers,scenario.Flags);
else
    indexer = GetIndexer(scenario.Thor,scenario.RoadNetwork,scenario.PowerNetwork,scenario.InitialConditions,scenario.RebWeight,scenario.Passengers,scenario.Flags);
end

if scenario.Flags.sourcerelaxflag
    eamod_problem.sourcerelaxflag = true;
    eamod_problem.SourceRelaxCost = lp_matrices.SourceRelaxCost;
end

spec = eamod_problem.spec;

f_cost_full_ref = lp_matrices.f_cost;
Ain_ref = lp_matrices.Aineq;
Bin_ref = lp_matrices.Bineq;
Aeq_ref = lp_matrices.Aeq;
Beq_ref = lp_matrices.Beq;
lb_StateVector_full_ref = lp_matrices.lb;
ub_StateVector_full_ref = lp_matrices.ub;

state_range = GetStateRange(eamod_problem,indexer);

% Cost vector
f_cost = eamod_problem.CreateCostVector();
f_cost_ref = f_cost_full_ref(state_range);

verifyEqual(test_case,f_cost,f_cost_ref)

if eamod_problem.use_real_time_formulation
    % CustomerChargeConservation
    [Aeq_CustomerChargeConservation, Beq_CustomerChargeConservation] = eamod_problem.CreateEqualityConstraintMatrices_CustomerChargeConservation();
    
    row_start_CustomerChargeConservation = indexer.FindEqCustomerChargeConservationktc(1,1,1);
    row_end_CustomerChargeConservation = indexer.FindEqCustomerChargeConservationktc(spec.n_passenger_flow,spec.n_time_step,spec.n_charge_step);
    row_range_CustomerChargeConservation = row_start_CustomerChargeConservation:row_end_CustomerChargeConservation;
    
    [Aeq_CustomerChargeConservation_ref,Beq_CustomerChargeConservation_ref] = ExtractConstraintSubmatrix(Aeq_ref,Beq_ref,row_range_CustomerChargeConservation,state_range);
    
    verifyEqualSparse(test_case,Aeq_CustomerChargeConservation,Aeq_CustomerChargeConservation_ref)
    verifyEqual(test_case,Beq_CustomerChargeConservation,Beq_CustomerChargeConservation_ref)
    
else
    % PaxConservation
    [Aeq_PaxConservation, Beq_PaxConservation] = eamod_problem.CreateEqualityConstraintMatrices_PaxConservation();
    
    row_start_PaxConservation = indexer.FindEqPaxConservationtcki(1,1,1,1);
    row_end_PaxConservation = indexer.FindEqPaxConservationtcki(spec.n_time_step,spec.n_charge_step,spec.n_passenger_flow,spec.n_road_node);
    row_range_PaxConservation = row_start_PaxConservation:row_end_PaxConservation;
    
    [Aeq_PaxConservation_ref,Beq_PaxConservation_ref] = ExtractConstraintSubmatrix(Aeq_ref,Beq_ref,row_range_PaxConservation,state_range);
    
    verifyEqualSparse(test_case,Aeq_PaxConservation,Aeq_PaxConservation_ref)
    verifyEqual(test_case,Beq_PaxConservation,Beq_PaxConservation_ref)
end
% RebConservation
[Aeq_RebConservation, Beq_RebConservation] = eamod_problem.CreateEqualityConstraintMatrices_RebConservation();

row_start_RebConservation = indexer.FindEqRebConservationtci(1,1,1);
row_end_RebConservation = indexer.FindEqRebConservationtci(spec.n_time_step,spec.n_charge_step,spec.n_road_node);
row_range_RebConservation = row_start_RebConservation:row_end_RebConservation;
[Aeq_RebConservation_ref,Beq_RebConservation_ref] = ExtractConstraintSubmatrix(Aeq_ref,Beq_ref,row_range_RebConservation,state_range);

verifyEqualSparse(test_case,Aeq_RebConservation,Aeq_RebConservation_ref)
verifyEqual(test_case,Beq_RebConservation,Beq_RebConservation_ref)

% SourceConservation
[Aeq_SourceConservation, Beq_SourceConservation] = eamod_problem.CreateEqualityConstraintMatrices_SourceConservation();

row_start_SourceConservation = indexer.FindEqSourceConservationks(1,1);
row_end_SourceConservation = indexer.FindEqSourceConservationks(spec.n_passenger_flow,length(spec.passenger_source_list_cell{end}));
row_range_SourceConservation = row_start_SourceConservation:row_end_SourceConservation;

[Aeq_SourceConservation_ref,Beq_SourceConservation_ref] = ExtractConstraintSubmatrix(Aeq_ref,Beq_ref,row_range_SourceConservation,state_range);

verifyEqualSparse(test_case,Aeq_SourceConservation,Aeq_SourceConservation_ref)
verifyEqual(test_case,Beq_SourceConservation,Beq_SourceConservation_ref)

% SinkConservation
[Aeq_SinkConservation, Beq_SinkConservation] = eamod_problem.CreateEqualityConstraintMatrices_SinkConservation();

row_start_SinkConservation = indexer.FindEqSinkConservationk(1);
row_end_SinkConservation = indexer.FindEqSinkConservationk(spec.n_passenger_sink);
row_range_SinkConservation = row_start_SinkConservation:row_end_SinkConservation;

[Aeq_SinkConservation_ref,Beq_SinkConservation_ref] = ExtractConstraintSubmatrix(Aeq_ref,Beq_ref,row_range_SinkConservation,state_range);

verifyEqualSparse(test_case,Aeq_SinkConservation,Aeq_SinkConservation_ref)
verifyEqual(test_case,Beq_SinkConservation,Beq_SinkConservation_ref)

% RoadCongestion
[Ain_RoadCongestion, Bin_RoadCongestion] = eamod_problem.CreateInequalityConstraintMatrices_RoadCongestion();

row_start_RoadCongestion = indexer.FindInRoadCongestiontij(1,1,1);
row_end_RoadCongestion = indexer.FindInRoadCongestiontij(spec.n_time_step,spec.n_road_node,spec.road_adjacency_list{end}(end));
row_range_RoadCongestion = row_start_RoadCongestion:row_end_RoadCongestion;

[Ain_RoadCongestion_ref,Bin_RoadCongestion_ref] = ExtractConstraintSubmatrix(Ain_ref,Bin_ref,row_range_RoadCongestion,state_range);

verifyEqualSparse(test_case,Ain_RoadCongestion,Ain_RoadCongestion_ref)
verifyEqual(test_case,Bin_RoadCongestion,Bin_RoadCongestion_ref,'RelTol',test_case.TestData.rel_tol_equality_hard)

% ChargerCongestion
[Ain_ChargerCongestion, Bin_ChargerCongestion] = eamod_problem.CreateInequalityConstraintMatrices_ChargerCongestion();

row_start_ChargerCongestion = indexer.FindInChargerCongestiontl(1,1);
row_end_ChargerCongestion = indexer.FindInChargerCongestiontl(spec.n_time_step,spec.n_charger);
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

function OptimizationResultsMatch(test_case,eamod_problem,fval_ref)
% We use Mosek's overload of linprog as in
% TVPowerBalancedFlow_withpower_sinkbundle.
eamod_problem.yalmip_settings = sdpsettings('solver','linprog');

[objective_value,solver_time,diagnostics] = eamod_problem.Solve();

% Our real-time objective value includes cost of pre-routed customer carrying 
% vehicles, AMoD-power implementation does not. Hence, we subtract it
if eamod_problem.use_real_time_formulation
    [~, pax_cost_val_usd, ~,relax_cost_val_usd] = eamod_problem.EvaluateAMoDcost();
    objective_value = objective_value - pax_cost_val_usd;
end

objective_value_ref = fval_ref;

verifyEqual(test_case,objective_value,objective_value_ref,'RelTol',test_case.TestData.rel_tol_equality_hard);
end




