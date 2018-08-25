function scenario = LoadScenario(data_path)
% LoadScenario Reads legacy data used in the AMoD-power implementation, preprocesses them and collects the results in a struct 

% This was copied from RSS17_ATX_TV_tract_sinkbundle_nb on March 5, 2018

%num_vehicles=4.5*1e5;
%Thor=2; %5 to 10 am

%----LOAD DATA---
load(data_path)

num_nodes = length(RoadGraph);
C = double(C);

%disp('Overriding minendcharge and roadcap and starttimes')
%MinEndCharge=0
%RoadCap=RoadCap*1000
%StartTimes=StartTimes-1
%ValueOfTime=12.2;

% initial distribution

init_distr = [];
for i=1:num_nodes
    init_distr(i) = sum(FlowsIn(Sources==i & StartTimes==1));
end
init_distr = init_distr / sum(init_distr);

% Modify source data. Note that for the TV version we'd want to keep unique
% (source, start time) pairs

CompactSinks=unique(Sinks);
CompactSinks=CompactSinks'; %We like column vectors
CompactSources=cell(length(CompactSinks),1);
CompactStartTimes=CompactSources;
CompactFlows=CompactSources;
for i=1:length(CompactSinks)
    CompactSources{i}=Sources(Sinks==CompactSinks(i));
    CompactStartTimes{i}=StartTimes(Sinks==CompactSinks(i));
    CompactFlows{i}=FlowsIn(Sinks==CompactSinks(i));
end

%Sources=CompactSources;
%Sinks=CompactSinks;
%StartTimes=CompactStartTimes;
%Flows=CompactFlows;

fprintf('There are %d customers\n',sum(FlowsIn));


disp('Overriding v2g efficiency')
v2g_efficiency=0;

fprintf('%d\n',num_nodes);
try
    num_vehicles>0;
catch
    num_vehicles=1e5;
end
% disp('OVERRIDE number of vehicles')
% num_vehicles=150000;

% Overrides
%num_vehicles=4*1e5;
%ChargerList=[1]
%ChargerSpeeds=[5]
%ChargerTimes=[1]
%ChargerCap=[0]
%MinEndCharge=C
%ChargeReqs=zeros(size(ChargeReqs));
%TravelTimes=TravelTimes>0;

%RoadToPowerMap=RoadToPowerMap(1)
%PowerToRoadMap=zeros(size(PowerToRoadMap));


%%
% Road data
RoadNetwork.C=C;
RoadNetwork.RoadGraph=RoadGraph;
RoadNetwork.RoadCap=RoadCap;
RoadNetwork.TravelTimes=TravelTimes;
RoadNetwork.TravelDistance=TravelDistance;
RoadNetwork.ChargeToTraverse=ChargeReqs;
RoadNetwork.ChargersList=ChargerList;
RoadNetwork.ChargerSpeed=ChargerSpeeds;
RoadNetwork.ChargerTime=ChargerTimes;
RoadNetwork.ChargeUnitToPowerUnit=ChargeUnitToPowerUnit;
RoadNetwork.ChargerCap=ChargerCaps;
RoadNetwork.MinEndCharge=MinEndCharge;
RoadNetwork.ValueOfTime=ValueOfTime;
RoadNetwork.VehicleCostPerKm=VehicleCostPerKm;

assert(all(size(ChargerSpeeds) == size(ChargerList)))
assert(all(size(ChargerList) == size(ChargerTimes)))

RoadNetworkEmpty.C=C;
RoadNetworkEmpty.RoadGraph={};
RoadNetworkEmpty.RoadCap=[];
RoadNetworkEmpty.TravelTimes=[];
RoadNetworkEmpty.TravelDistance=[];
RoadNetworkEmpty.ChargeToTraverse=[];
RoadNetworkEmpty.ChargersList=[];
RoadNetworkEmpty.ChargerSpeed=[];
RoadNetworkEmpty.ChargerTime=[];
RoadNetworkEmpty.ChargeUnitToPowerUnit=1;
RoadNetworkEmpty.ChargerCap=[];
RoadNetworkEmpty.MinEndCharge=1;
RoadNetworkEmpty.ValueOfTime=0;
RoadNetworkEmpty.VehicleCostPerKm=0;



% Power Data
%PowerGraph=cell(n_chargers,1);
%PowerLineCap=Inf(length(PowerGraph));

PowerNetwork.PowerGraphM=PowerGraphFull;
PowerNetwork.PowerLineCapM=PowerLineCapFull;
PowerNetwork.PowerLineReactanceM=PowerLineReactanceFull;
%PowerNetwork.PowerGraph=PowerGraph;
%PowerNetwork.PowerLineCap=PowerLineCap;
%PowerNetwork.PowerLineReactance=PowerLineReactance;
PowerNetwork.PowerGensList=PowerGensList;
PowerNetwork.PowerGensMax=PowerGensMax';
PowerNetwork.PowerGensMin=PowerGensMin';
PowerNetwork.PowerCosts=PowerCosts';
PowerNetwork.PowerExtLoads=PowerExtLoads;
PowerNetwork.RoadToPowerMap=RoadToPowerMap;
%PowerNetwork.PowerToRoadMap=PowerToRoadMap;
PowerNetwork.v2g_efficiency=v2g_efficiency;
PowerNetwork.PowerRampUp=PowerRampUp;
PowerNetwork.PowerRampDown=PowerRampDown;

% Flow data
M=length(CompactSources);
Sources=CompactSources;
Sinks=CompactSinks;
FlowsIn=CompactFlows;
StartTimes=CompactStartTimes;

% The block below solves for the power network only
% Sources=[1]
% Sinks={[1]}
% FlowsIn={[0]}
% StartTimes=[1]
% num_vehicles=0

Passengers.Sources=Sources;
Passengers.Sinks=Sinks;
Passengers.Flows=FlowsIn;
Passengers.StartTimes=StartTimes;


PassengersEmpty.Sources={[1]};
PassengersEmpty.Sinks=[1];
PassengersEmpty.Flows={[0]};
PassengersEmpty.StartTimes={[1]};

% SET INITIAL CONDITIONS
InitialRebPos=zeros(num_nodes, C);
%InitialRebPos(CompactSources,C/2)=num_vehicles/length(CompactSources) *ones(length(CompactSources),1);
InitialRebPos(:,MinEndCharge)=num_vehicles * init_distr;
InitialPaxPos=zeros(M,num_nodes,C);

InitialConditions.FullVehicleInitialPos=InitialPaxPos;
InitialConditions.EmptyVehicleInitialPos=InitialRebPos;

% OPTIMIZATION CONFIG
milpflag=0;
congrelaxflag=0;
sourcerelaxflag=0;
cachedAeqflag=0;

Flags.milpflag=milpflag;
Flags.congrelaxflag=congrelaxflag;
Flags.sourcerelaxflag=sourcerelaxflag;
Flags.cachedAeqflag=cachedAeqflag;
Flags.solverflag='MOSEK';

RebWeight=0.2;

% We add a few fields for the real-time optimizer

%     - TVRoadCap, a txnxn matrix. RoadCap(t,i,j) is the capacity of the i-j link
%        (in vehicles per unit time) at time t.
N=length(RoadNetwork.RoadGraph);

TVRoadCap=zeros(Thor,N,N);
for tt=1:Thor
    TVRoadCap(tt,:,:)=RoadNetwork.RoadCap;
end
RoadNetwork.TVRoadCap=TVRoadCap;



%     - EmptyVehicleInitialPosRT, a ThorxNxC matrix encoding the initial
%        position and charge level of the vehicles. EVIP-RT(t,i,c) is the
%        number of empty vehicles at node i and charge c at time t.

EmptyVehicleInitialPosRT = zeros(Thor,N,C);
EmptyVehicleInitialPosRT(1,:,:)=InitialConditions.EmptyVehicleInitialPos;
InitialConditions.EmptyVehicleInitialPosRT=EmptyVehicleInitialPosRT;

% Build scenario
scenario.Thor = Thor;

scenario.RoadNetwork = RoadNetwork;
scenario.RoadNetworkEmpty = RoadNetworkEmpty;

scenario.PowerNetwork = PowerNetwork;

scenario.InitialConditions = InitialConditions;

scenario.RebWeight = RebWeight;

scenario.Passengers = Passengers;
scenario.PassengersEmpty = PassengersEmpty;

scenario.Flags = Flags;

% Extra data
scenario.time_step_s = double(timeStepSize);
scenario.charge_unit_j = ChargeUnitToPowerUnit*scenario.time_step_s*BaseMVA*1e6;
scenario.BaseMVA = BaseMVA;

scenario = AdaptScenarioForRealTime(scenario);

end


function scenario = AdaptScenarioForRealTime(scenario)
% Unpack scenario
Passengers = scenario.Passengers;
RoadNetwork = scenario.RoadNetwork;
Thor = scenario.Thor;

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

% Add routes for real-time formulation
[RouteTime,RouteCharge,Routes] = build_routes(RoadNetwork.RoadGraph,RoadNetwork.TravelTimes,RoadNetwork.ChargeToTraverse);

RoadNetwork.RouteTime = RouteTime;
RoadNetwork.RouteCharge = RouteCharge;

TVRoadCap = RoadNetwork.TVRoadCap;
% This was taken from Nature_MATLAB_driver_fun
for sinkid = 1:length(Passengers.Sinks)    
    for sourceid = 1:length(Passengers.Sources{sinkid})
        source = Passengers.Sources{sinkid}(sourceid);
        sink = Passengers.Sinks(sinkid);
        starttime = Passengers.StartTimes{sinkid}(sourceid);
        flow = Passengers.Flows{sinkid}(sourceid);
        route = Routes{source,sink};
        mytime = starttime;
        for edgeid = 1:length(route)-1
            route(edgeid:edgeid+1);
            edge = route(edgeid:edgeid+1);
            edgetime = RoadNetwork.TravelTimes(edge(1),edge(2));
            assert(edgetime>0,'ERROR, is this an edge?');
            finaledgetime = min(mytime+edgetime-1,Thor);
            TVRoadCap(mytime:finaledgetime,edge(1),edge(2))= TVRoadCap(mytime:finaledgetime,edge(1),edge(2))-flow;
            mytime=mytime+edgetime;
        end
        assert(mytime == starttime + RouteTime(source,sink),'WEIRD TIME')
    end
end

RoadNetwork.TVRoadCap = TVRoadCap;
RoadNetwork.RouteTime = RouteTime;
RoadNetwork.RouteCharge = RouteCharge;

scenario.RoadNetwork = RoadNetwork;


end

function [RouteTime,RouteCharge,Routes] = build_routes(RoadGraph,TravelTimes,ChargeToTraverse)

N=length(RoadGraph);

RouteTime=zeros(N);
RouteCharge=zeros(N);
Routes=cell(N,N);

for i=1:N
    for j=1:N
        Routes{i,j} = FreeRouteAstar(i,j,RoadGraph,TravelTimes);
        for k=1:length(Routes{i,j})-1
            RouteTime(i,j)=RouteTime(i,j)+TravelTimes(Routes{i,j}(k),Routes{i,j}(k+1));
            RouteCharge(i,j)=RouteCharge(i,j)+ChargeToTraverse(Routes{i,j}(k),Routes{i,j}(k+1));
        end
    end
end

end

function [Route] = FreeRouteAstar(start,goal,RoadGraph,RoadCost,heuristic_est)
N=length(RoadGraph);
if nargin<=4
    heuristic_est = zeros(N); %revert to Dijkstra
end

%forked from the AMoD-congestion implementation

ClosedSet = [];
OpenSet = [start];

CameFrom=-1*ones(N);
g_score = Inf*ones(N,1);
f_score = Inf*ones(N,1);
g_score(start)=0;

f_score(start)=g_score(start)+heuristic_est(start,goal);

Route = [];

while ~isempty(OpenSet)
    [~,currInOpenSet]=min(f_score(OpenSet));
    currNode = OpenSet(currInOpenSet);
    if currNode == goal
        
        
        Route = goal;
        while (Route(1)~=start)
            Route=[CameFrom(Route(1)),Route];
        end
        
        return
    end
    OpenSet = OpenSet(OpenSet~=currNode);       %Remove current node
    ClosedSet = [ClosedSet, currNode];          %Add to closed set
    for neigh = RoadGraph{currNode}
        if ~sum(ClosedSet == neigh)             %If neighbor is not in closed set
            tentative_g = g_score(currNode) + RoadCost(currNode,neigh);
            if (~sum(neigh == OpenSet))         %If the neighbor is not already in OpenSet, add it
                OpenSet = [OpenSet, neigh];
            elseif tentative_g>=g_score(neigh)
                continue
            end
            CameFrom(neigh) = currNode;
            g_score(neigh) = tentative_g;
            f_score(neigh) = g_score(neigh)+heuristic_est(neigh,goal);
            
        end
    end
end

end

