function indexer = GetIndexer(Thor,RoadNetwork,PowerNetwork,InitialConditions,RebWeight,Passengers,Flags)
% GetIndexer Creates an indexer to relate constraints in the AMoD-power implementation to their rows in the matrices of the linear program

TVPowerBalancedFlowFinder_sinkbundle;

% Create constraint indexers
indexer.FindEqPaxConservationtcki = @(t,c,k,i) N*M*C*(t - 1) + N*M*(c - 1) + N*(k - 1) + i;
indexer.FindEqRebConservationtci = @(t,c,i) indexer.FindEqPaxConservationtcki(Thor,C,M,N) + N*C*(t - 1) + N*(c - 1) + i;
indexer.FindEqSourceConservationks = @(k,s) indexer.FindEqRebConservationtci(Thor,C,N) + CumNumSourcesPerSink(k) + s;
indexer.FindEqSinkConservationk = @(k) indexer.FindEqSourceConservationks(M,length(Sources{end})) + k;

indexer.FindInRoadCongestiontij = @(t,i,j) E*(t - 1) + cumRoadNeighbors(i) + RoadNeighborCounter(i,j);
indexer.FindInChargerCongestiontl = @(t,l) indexer.FindInRoadCongestiontij(Thor,N,RoadGraph{end}(end)) + NumChargers*(t-1) + l;

% Add indexers from TVPowerBalancedFlowFinder_sinkbundle
indexer.FindRoadLinkPtckij = FindRoadLinkPtckij;
indexer.FindRoadLinkRtcij = FindRoadLinkRtcij;
indexer.FindChargeLinkPtckl = FindChargeLinkPtckl;
indexer.FindChargeLinkRtcl = FindChargeLinkRtcl;
indexer.FindDischargeLinkPtckl = FindDischargeLinkPtckl;
indexer.FindDischargeLinkRtcl = FindDischargeLinkRtcl;

indexer.FindPaxSourceChargecks = FindPaxSourceChargecks;
indexer.FindPaxSinkChargetck = FindPaxSinkChargetck;

indexer.FindEndRebLocationci = FindEndRebLocationci;

indexer.FindSourceRelaxks = FindSourceRelaxks;
end