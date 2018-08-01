classdef EAMoDspec
    methods
        function obj = EAMoDspec()
        end
        
        % It is sufficient to look at the initial conditions because
        % vehicles are conserved
        function n_vehicle = get.n_vehicle(obj)
            n_vehicle_full_init = sum(obj.InitialConditions.FullVehicleInitialPos(:));
            n_vehicle_empty_init = sum(obj.InitialConditions.EmptyVehicleInitialPos(:));
            
            n_vehicle = n_vehicle_full_init + n_vehicle_empty_init;
        end
        
        % This is for convenience and compatibility with other specs
        function n_time_step = get.n_time_step(obj)
            n_time_step = obj.Thor;
        end
        
        function VerifySpec(obj)
            assert(all(size(obj.RoadNetwork.ChargerSpeeds) == size(obj.RoadNetwork.ChargerList)))
            assert(all(size(obj.RoadNetwork.ChargerList) == size(obj.RoadNetwork.ChargerTimes)))
            
        end
        
        
    end
    
    % * Thor, the optimization horizon (an int)
    %
    % * Inside RoadNetwork
    % - C, a scalar. The overall number of charge levels.
    % - RoadGraph, a nx1 cell structure. RoadGraph{i} contains the neighbors of
    %    i in the road graph
    % - RoadCap, a nxn matrix. RoadCap(i,j) is the capacity of the i-j link
    %    (in vehicles per unit time).
    % - TravelTimes, a nxn matrix of ints. TravelTimes(i,j) is the travel time
    %    along the i-j link.
    % - TravelDistance, a nxn matrix of floats. TravelDistance(i,j) is the
    %    travel distance along the i-j link.
    % - ChargeToTraverse, a nxn matrix. ChargeReqs(i,j) is the amount of (quantized)
    %    units of charge required to travel from i to j. For a charge link,
    % - ChargersList, a vector that identifies chargers. CL(i) is the location
    %    of the node in RoadGraph corresponding to the i-th charger
    % - ChargerSpeed, the charging speed (in charge units per time) of the
    %    chargers. CS(i) is the amount of charge gained by a vehicle crossing
    %    charge link i (corresponding to node CS(i)).
    % - ChargerTime, the time required to charge ChargerSpeed units of charger
    %    at charger i. CS(i) is the charge speed of charger i (at node CL(i)).
    % - ChargerCap, the maximum number of vehicles that a charger can service.
    % - ChargeUnitToEnergyUnit, a scalar denoting the conversion between charge
    %    units per unit time (in the AMoD problem) and energy units (in the
    %    power network problem).
    % - MinEndCharge, a scalar denoting the minimum charge (in charge levels)
    %    of the rebalancing vehicles at the end of the simulation. This
    %    value is exclusive. For example, setting it to 1 requires a
    %    minumum charge of 2.
    % 
    % - ValueOfTime, the value (in USD) of one unit of time for a passenger
    % - VehicleCostPerKm, the cost of running a vehicle for 1km
    %
    % * Inside Passengers
    % - Sinks, a m-x-1 vector. Sinks(i) is the sink node of the i-th
    %    flow.
    % - Sources, a m-x-1 cell. Sources{i} is a vector. Sources{i}(j) is the
    %    j-th origin leaving at time StartTimes{i}(j) for sink Sinks(i).
    % - StartTimes, same as Sources. ST{i}(j) is the departure time from
    %    source Sources{i}(j).
    % - Flows, a m-by-1 cell. FlowsIn{i}(j) is the amount of (vehicle) flow
    %    entering the source of flow Sources{i}(j) at time ST{i}(j) bound for
    %    Sinks(i).    
    %
    % * Inside InitialConditions
    % - FullVehicleInitialPos, a MXNxC matrix encoding the initial position and
    %    charge level of the vehicles. FVIP(m,i,c) is the number of vehicles
    %    with pax M at node i and charge c at time t=1.
    % - EmptyVehicleInitialPos, a NxC matrix encoding the initial position and
    %    charge level of the vehicles. EVIP(i,c) is the number of empty vehicles
    %    at node i and charge c at time t=1.
    %
    % * Inside Flags
    % - sourcerelaxflag (default: 0). %If this flag is on, each vehicle flow is
    %    allowed to reduce its sources (and sinks) for a cost. This is
    %    especially useful when it is not possible to compute a satisfying
    %    rebalancing flow because of timing constraints but, if the congestion
    %    constraints are tight, it can also preserve feasibility if not all
    %    flows can be realized. A slack variable is defined for each source and sink.
    
    % Extras
    % time_step_s, charge_unit_j, v2g_efficiency
    % Old: ChargeUnitToPowerUnit
    % solver
    % start_date_time
    
    properties
        Thor
        RoadNetwork
        InitialConditions
        Passengers
        Flags
        time_step_s
        charge_unit_j
        v2g_efficiency
        ChargeUnitToPowerUnit
        solver
        
        start_date_time(1,1) datetime = datetime('01-Jan-2010 00:00:00');
    end
    
    
    properties (Dependent)
       n_vehicle 
       n_time_step
    end
    
    
end