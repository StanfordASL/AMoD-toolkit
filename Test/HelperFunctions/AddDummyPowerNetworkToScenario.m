function scenario = AddDummyPowerNetworkToScenario(scenario)

numChargers = length(scenario.RoadNetwork.ChargersList);
power_costs = zeros(numChargers,scenario.Thor)';
PowerNetwork_dummy = CreateDummyPowerNetwork(scenario.Thor,numChargers,scenario.PowerNetwork.v2g_efficiency,power_costs);
scenario.PowerNetwork = PowerNetwork_dummy;
end