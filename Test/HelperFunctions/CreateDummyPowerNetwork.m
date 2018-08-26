function PowerNetworkR = CreateDummyPowerNetwork(Thor,numChargers,v2g_efficiency,power_costs)
% CreateDummyPowerNetwork Creates a dummy power network with an unbounded generator at every node with power priced according to power_costs
%   PowerNetworkR = CreateDummyPowerNetwork(Thor,numChargers,v2g_efficiency,power_costs)
%   Thor is the number of time-steps, numChargers is the number of chargers,
%   v2g_efficiency is the efficency of v2g and
%   power_costs(t,l) is the power cost in USD/(baseMVA*time_step) of charger l at timestep t   

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