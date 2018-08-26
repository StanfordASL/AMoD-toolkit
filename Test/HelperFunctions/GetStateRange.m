function state_range = GetStateRange(eamod_problem,indexer)
% GetStateRange Returns the range of columns in legacy implementation that corresponds to the state in EAMoDproblem
%
%   See also GetIndexer, GetIndexerRealTime

spec = eamod_problem.spec;

if eamod_problem.use_real_time_formulation
    state_range = indexer.FindRoadLinkRtcij(1,1,1,1):indexer.FindEndRebLocationci(spec.n_charge_step,spec.n_road_node);
else
    state_range = indexer.FindRoadLinkPtckij(1,1,1,1,1):indexer.FindEndRebLocationci(spec.n_charge_step,spec.n_road_node);
end

if eamod_problem.sourcerelaxflag
    col_range_relax = indexer.FindSourceRelaxks(1,1) + [0:(spec.n_passenger_source - 1)];
    state_range = [state_range,col_range_relax];
end
end
