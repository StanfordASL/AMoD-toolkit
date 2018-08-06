function obj = CreateFromScenario(scenario)
RoadNetwork = scenario.RoadNetwork;
InitialConditions = scenario.InitialConditions;
Passengers = scenario.Passengers;
Flags = scenario.Flags;

obj = EAMoDspec();

obj.Thor = scenario.Thor;

obj.C = RoadNetwork.C;

obj.RoadGraph = RoadNetwork.RoadGraph;

obj.Sources = Passengers.Sources;
obj.Sinks = Passengers.Sinks;
obj.Flows = Passengers.Flows;
obj.StartTimes = Passengers.StartTimes;

obj.RoadCap = RoadNetwork.RoadCap;
obj.TravelTimes = RoadNetwork.TravelTimes;
obj.TravelDistance = RoadNetwork.TravelDistance;

obj.ChargersList = RoadNetwork.ChargersList;
obj.ChargerTime = RoadNetwork.ChargerTime;

obj.ChargeToTraverse = RoadNetwork.ChargeToTraverse;
obj.ChargerSpeed = RoadNetwork.ChargerSpeed;
obj.ChargerCap = RoadNetwork.ChargerCap;

% Initial conditions
obj.FullVehicleInitialPos = InitialConditions.FullVehicleInitialPos;
obj.EmptyVehicleInitialPos = InitialConditions.EmptyVehicleInitialPos;

% Final conditions
obj.MinEndCharge = RoadNetwork.MinEndCharge;

% Economic settings
obj.ValueOfTime = RoadNetwork.ValueOfTime;
obj.VehicleCostPerM = RoadNetwork.VehicleCostPerKm/1000;

% Relaxation
if Flags.congrelaxflag
    warning('congrelaxflag is set but functionality is not available yet. Ignoring.')
end

obj.sourcerelaxflag = Flags.sourcerelaxflag;

% Stuff for real time algorithm
obj.TVRoadCap = zeros(obj.Thor,obj.N,obj.N);
for tt = 1:obj.Thor
    obj.TVRoadCap(tt,:,:) = obj.RoadCap;
end

% Extras
obj.v2g_efficiency = scenario.PowerNetwork.v2g_efficiency;

if isfield(scenario.RoadNetwork,'BatteryDepreciationPerUnitCharge')
    obj.BatteryDepreciationPerUnitCharge = scenario.RoadNetwork.BatteryDepreciationPerUnitCharge;
end
    
    
end