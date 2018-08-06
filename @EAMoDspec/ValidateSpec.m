function ValidateSpec(obj)
% ValidateSpec Asserts that the properties of the EAMoDspec object are consistent with eachother

% Properties that depend on RoadGraph
obj.ValidateConsistencyWithAdjancencyMatrix(obj.RoadCap,'RoadCap');
obj.ValidateConsistencyWithAdjancencyMatrix(obj.TravelTimes,'TravelTimes');
obj.ValidateConsistencyWithAdjancencyMatrix(obj.TravelDistance,'TravelDistance');
obj.ValidateConsistencyWithAdjancencyMatrix(obj.ChargeToTraverse,'ChargeToTraverse');

% Properties that depend on ChargersList
assert(numel(obj.ChargerSpeed) == obj.NumChargers,'ChargerSpeed must have NumChargers elements.');
assert(numel(obj.ChargerTime) == obj.NumChargers,'ChargerTime must have NumChargers elements');
assert(numel(obj.ChargerTime) == obj.NumChargers,'ChargerCap must have NumChargers elements');

% Properties that depend on Sinks
assert(numel(obj.Sources) == obj.NumSinks,'Sources must have NumSinks elements');

% Properties that depend on Sources
assert(all(cellfun(@numel,obj.StartTimes) == obj.NumSourcesPerSink),'StartTimes must have the same number of cells and number of elements per cell as Sources');
assert(all(cellfun(@(x) all(x >= 0) & all(x <= obj.Thor),obj.StartTimes)),'All StartTimes must be in range [0,Thor]');

assert(all(cellfun(@numel,obj.Flows) == obj.NumSourcesPerSink),'Flows must have the same number of cells and number of elements per cell as Sources');
assert(all(cellfun(@(x) all(x >= 0),obj.Flows)),'All Flows must be non-negative');

% Entries that must be valid road nodes
assert(all(obj.IsValidRoadNode(obj.ChargersList)),'Entries in ChargersList must be valid road nodes');
assert(all(obj.IsValidRoadNode(obj.Sinks)),'Entries in Sinks must be valid road nodes');

for i_sink = 1:obj.NumSinks
    sources_towards_sink = obj.Sources{i_sink};
    assert(all(obj.IsValidRoadNode(sources_towards_sink)),'Entries in Sources{%i} must be valid road nodes',i_sink);
end

% Others
assert(obj.MinEndCharge <= obj.C - 1,'MinEndCharge may not be more than C - 1 (recall it is exclusive)');

assert(all(size(obj.FullVehicleInitialPos) == [obj.M,obj.N,obj.C]),'FullVehicleInitialPos must be of size M x N x C');
assert(all(size(obj.EmptyVehicleInitialPos) == [obj.N,obj.C]),'FullVehicleInitialPos must be of size N x C');


end