function state_range = GetStateRange(eamod_problem,indexer)
spec = eamod_problem.spec;

if eamod_problem.use_real_time_formulation
    state_range = indexer.FindRoadLinkRtcij(1,1,1,1):indexer.FindEndRebLocationci(spec.C,spec.N);
else
    state_range = indexer.FindRoadLinkPtckij(1,1,1,1,1):indexer.FindEndRebLocationci(spec.C,spec.N);
end

if eamod_problem.sourcerelaxflag
    col_range_relax = indexer.FindSourceRelaxks(1,1) + [0:(spec.TotNumSources - 1)];
    state_range = [state_range,col_range_relax];
end
end
