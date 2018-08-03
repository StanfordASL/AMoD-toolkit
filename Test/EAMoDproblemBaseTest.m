function tests = EAMoDproblemBaseTest
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
mosek_path = 'C:/Program Files/Mosek/8/toolbox/r2014a';
addpath(genpath(mosek_path));

amod_power_path = '../../AMoD-power/';
addpath(amod_power_path);

test_case.TestData.rel_tol_equality = 1e-6;

test_case.TestData.data_path_cell = {'dfw_roadgraph_kmeans_tv_federico_5cl_windsun_12h_v3_feas'};
end

% This test assumes that TVPowerBalancedFlow_withpower_sinkbundle matches TVPowerBalancedFlow_withpower_sinkbundle_legacy
% This is tested by TVPowerBalancedFlow_withpower_sinkbundleTest.
function TestProblemMatricesMatch(test_case)

n_data_path = numel(test_case.TestData.data_path_cell);

for i_data_path = 1:n_data_path
    data_path = test_case.TestData.data_path_cell{i_data_path};
    
    scenario = LoadScenario(data_path);    
    ProblemMatricesMatchHelper(test_case,scenario);
    
    % Test with relaxed source constraints
    scenario_sourcerelaxflag = scenario;
    scenario_sourcerelaxflag.Flags.sourcerelaxflag = true;    
    ProblemMatricesMatchHelper(test_case,scenario_sourcerelaxflag);
    
end
end

function ProblemMatricesMatchHelper(test_case,scenario)
spec = EAMoDspec.CreateFromScenario(scenario);

[f_cost_full_ref,Ain_ref,Bin_ref,Aeq_ref,Beq_ref,lb_StateVector_full_ref,ub_StateVector_full_ref,~,~,indexer,SourceRelaxCost] = ...
    TVPowerBalancedFlow_withpower_sinkbundle_ConstraintMatrices(scenario.Thor,scenario.RoadNetwork,scenario.PowerNetwork,scenario.InitialConditions,scenario.RebWeight,scenario.Passengers,scenario.Flags);

col_range = indexer.FindRoadLinkPtckij(1,1,1,1,1):indexer.FindEndRebLocationci(spec.C,spec.N);

if scenario.Flags.sourcerelaxflag
    col_range_relax = indexer.FindSourceRelaxks(1,1) + [0:(spec.TotNumSources - 1)];
    col_range = [col_range,col_range_relax];
    
    % This is hardcoded in TVPowerBalancedFlowFinder_sinkbundle
    spec.SourceRelaxCost = SourceRelaxCost;
end

eamod_problem = EAMoDproblemBase(spec);


% Cost vector
f_cost = eamod_problem.CreateCostVector();
f_cost_ref = f_cost_full_ref(col_range);

verifyEqual(test_case,f_cost,f_cost_ref)

% PaxConservation
[Aeq_PaxConservation, Beq_PaxConservation] = eamod_problem.CreateEqualityConstraintMatrices_PaxConservation();

row_start_PaxConservation = indexer.FindEqPaxConservationtcki(1,1,1,1);
row_end_PaxConservation = indexer.FindEqPaxConservationtcki(spec.Thor,spec.C,spec.M,spec.N);
row_range_PaxConservation = row_start_PaxConservation:row_end_PaxConservation;

[Aeq_PaxConservation_ref,Beq_PaxConservation_ref] = ExtractConstraintSubmatrix(Aeq_ref,Beq_ref,row_range_PaxConservation,col_range);

verifyEqualSparse(test_case,Aeq_PaxConservation,Aeq_PaxConservation_ref)
verifyEqual(test_case,Beq_PaxConservation,Beq_PaxConservation_ref)

% RebConservation
[Aeq_RebConservation, Beq_RebConservation] = eamod_problem.CreateEqualityConstraintMatrices_RebConservation();

row_start_RebConservation = indexer.FindEqRebConservationtci(1,1,1);
row_end_RebConservation = indexer.FindEqRebConservationtci(spec.Thor,spec.C,spec.N);
row_range_RebConservation = row_start_RebConservation:row_end_RebConservation;
[Aeq_RebConservation_ref,Beq_RebConservation_ref] = ExtractConstraintSubmatrix(Aeq_ref,Beq_ref,row_range_RebConservation,col_range);

verifyEqualSparse(test_case,Aeq_RebConservation,Aeq_RebConservation_ref)
verifyEqual(test_case,Beq_RebConservation,Beq_RebConservation_ref)

% SourceConservation
[Aeq_SourceConservation, Beq_SourceConservation] = eamod_problem.CreateEqualityConstraintMatrices_SourceConservation();

row_start_SourceConservation = indexer.FindEqSourceConservationks(1,1);
row_end_SourceConservation = indexer.FindEqSourceConservationks(spec.M,length(spec.Sources{end}));
row_range_SourceConservation = row_start_SourceConservation:row_end_SourceConservation;

[Aeq_SourceConservation_ref,Beq_SourceConservation_ref] = ExtractConstraintSubmatrix(Aeq_ref,Beq_ref,row_range_SourceConservation,col_range);

verifyEqualSparse(test_case,Aeq_SourceConservation,Aeq_SourceConservation_ref)
verifyEqual(test_case,Beq_SourceConservation,Beq_SourceConservation_ref)

% SinkConservation
[Aeq_SinkConservation, Beq_SinkConservation] = eamod_problem.CreateEqualityConstraintMatrices_SinkConservation();

row_start_SinkConservation = indexer.FindEqSinkConservationk(1);
row_end_SinkConservation = indexer.FindEqSinkConservationk(spec.NumSinks);
row_range_SinkConservation = row_start_SinkConservation:row_end_SinkConservation;

[Aeq_SinkConservation_ref,Beq_SinkConservation_ref] = ExtractConstraintSubmatrix(Aeq_ref,Beq_ref,row_range_SinkConservation,col_range);

verifyEqualSparse(test_case,Aeq_SinkConservation,Aeq_SinkConservation_ref)
verifyEqual(test_case,Beq_SinkConservation,Beq_SinkConservation_ref)

% RoadCongestion
[Ain_RoadCongestion, Bin_RoadCongestion] = eamod_problem.CreateInequalityConstraintMatrices_RoadCongestion();

row_start_RoadCongestion = indexer.FindInRoadCongestiontij(1,1,1);
row_end_RoadCongestion = indexer.FindInRoadCongestiontij(spec.Thor,spec.N,spec.RoadGraph{end}(end));
row_range_RoadCongestion = row_start_RoadCongestion:row_end_RoadCongestion;

[Ain_RoadCongestion_ref,Bin_RoadCongestion_ref] = ExtractConstraintSubmatrix(Ain_ref,Bin_ref,row_range_RoadCongestion,col_range);

verifyEqualSparse(test_case,Ain_RoadCongestion,Ain_RoadCongestion_ref)
verifyEqual(test_case,Bin_RoadCongestion,Bin_RoadCongestion_ref)

% ChargerCongestion
[Ain_ChargerCongestion, Bin_ChargerCongestion] = eamod_problem.CreateInequalityConstraintMatrices_ChargerCongestion();

row_start_ChargerCongestion = indexer.FindInChargerCongestiontl(1,1);
row_end_ChargerCongestion = indexer.FindInChargerCongestiontl(spec.Thor,spec.NumChargers);
row_range_ChargerCongestion = row_start_ChargerCongestion:row_end_ChargerCongestion;

[Ain_ChargerCongestion_ref,Bin_ChargerCongestion_ref] = ExtractConstraintSubmatrix(Ain_ref,Bin_ref,row_range_ChargerCongestion,col_range);

verifyEqualSparse(test_case,Ain_ChargerCongestion,Ain_ChargerCongestion_ref)
verifyEqual(test_case,Bin_ChargerCongestion,Bin_ChargerCongestion_ref)

% StateVectorBounds
[lb_StateVector,ub_StateVector] = eamod_problem.CreateStateVectorBounds();

lb_StateVector_ref = lb_StateVector_full_ref(col_range);
ub_StateVector_ref = ub_StateVector_full_ref(col_range);

verifyEqual(test_case,lb_StateVector,lb_StateVector_ref)
verifyEqual(test_case,ub_StateVector,ub_StateVector_ref)

end