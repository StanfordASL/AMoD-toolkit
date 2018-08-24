% Creates a dummy power network with an unbounded generator at every node with power priced according to power_costs
function PowerNetworkR = CreateDummyPowerNetwork(Thor,numChargers,v2g_efficiency,power_costs)
% power_costs(t,l) is the power cost in USD/(baseMVA*time_step) of charger l at timestep t

PowerGraphR = cell(1,numChargers);
PowerLineCapR = PowerGraphR;
PowerLineReactanceR = PowerGraphR;
PowerGensListR = 1:numChargers;
PowerGensMaxR = Inf*ones(numChargers,Thor);
PowerGensMinR = -Inf*ones(numChargers,Thor);
PowerExtLoadsR = zeros(numChargers,Thor);
RoadToPowerMapR =  1:numChargers;

PowerNetworkR.PowerGraphM = PowerGraphR;
PowerNetworkR.PowerLineCapM = PowerLineCapR;
PowerNetworkR.PowerLineReactanceM = PowerLineReactanceR;
PowerNetworkR.PowerGensList = PowerGensListR;
PowerNetworkR.PowerGensMax = PowerGensMaxR';
PowerNetworkR.PowerGensMin = PowerGensMinR';
PowerNetworkR.PowerCosts = power_costs;
PowerNetworkR.PowerExtLoads = PowerExtLoadsR;
PowerNetworkR.RoadToPowerMap = RoadToPowerMapR;
PowerNetworkR.v2g_efficiency = v2g_efficiency;

end