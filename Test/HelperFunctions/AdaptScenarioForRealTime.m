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
