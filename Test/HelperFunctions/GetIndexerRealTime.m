function indexer = GetIndexerRealTime(Thor,RoadNetwork,PowerNetwork,InitialConditions,RebWeight,Passengers,Flags)
% GetIndexerRealTime Creates an indexer to relate constraints in the AMoD-power real-time implementation to their rows in the matrices of the linear program

TVPowerBalancedFlowFinder_realtime;

assert(MS == 0,'Starter flows are not supported.')

% Create constraint indexers
indexer.FindEqRebConservationtci = @(t,c,i) N*C*(t - 1) + N*(c - 1) + i;

indexer.FindEqSourceConservationks = @(k,s) indexer.FindEqRebConservationtci(Thor,C,N) + CumNumSourcesPerSink(k) + s;
indexer.FindEqSinkConservationk = @(k) indexer.FindEqSourceConservationks(M,length(Sources{end})) + k;

indexer.FindEqCustomerChargeConservationktc = @(k,t,c) indexer.FindEqSinkConservationk(M) + C*Thor*(k - 1) + C*(t-1) + c;

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