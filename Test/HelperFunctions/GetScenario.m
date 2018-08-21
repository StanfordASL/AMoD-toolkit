function scenario = GetScenario(data_path)
scenario = LoadScenario(data_path);    
    
numChargers = length(scenario.RoadNetwork.ChargersList);
power_costs = zeros(numChargers,scenario.Thor)';
PowerNetwork_dummy = CreateDummyPowerNetwork(scenario.Thor,numChargers,scenario.PowerNetwork.v2g_efficiency,power_costs);
scenario.PowerNetwork = PowerNetwork_dummy;

% Add routes for real-time formulation
[scenario.RoadNetwork.RouteTime,scenario.RoadNetwork.RouteCharge] = build_routes(scenario.RoadNetwork.RoadGraph,scenario.RoadNetwork.TravelTimes,scenario.RoadNetwork.ChargeToTraverse);
end