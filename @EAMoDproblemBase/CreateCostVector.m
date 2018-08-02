function [f_cost,f_cost_pax,f_cost_reb,f_cost_relax] = CreateCostVector(obj)
f_cost_pax = zeros(obj.StateSize,1);
f_cost_reb = zeros(obj.StateSize,1);
f_cost_relax = zeros(obj.StateSize,1);

% Passengers' and rebalancers' travel time and battery depreciation
for t = 1:obj.Thor
    for i = 1:obj.N
        for j = obj.RoadGraph{i}
            for c = 1:obj.C
                % Loop does not run for obj.num_passenger_flows = 0
                for k = 1:obj.num_passenger_flows
                    % Recall that TravelDistance is in meter
                    f_cost_pax(obj.FindRoadLinkPtckij(t,c,k,i,j)) = obj.ValueOfTime*obj.TravelTimes(i,j) + obj.TravelDistance(i,j)*obj.VehicleCostPerM + abs(obj.ChargeToTraverse(i,j))*obj.BatteryDepreciationPerUnitCharge;
                end
                if j ~= i
                    % Recall that TravelDistance is in meter
                    f_cost_reb(obj.FindRoadLinkRtcij(t,c,i,j)) = obj.TravelDistance(i,j)*obj.VehicleCostPerM + abs(obj.ChargeToTraverse(i,j))*obj.BatteryDepreciationPerUnitCharge;
                end
            end
        end
    end
end

% Passengers' and rebalancers' charging time and cost  plus battery depreciation
for t = 1:obj.Thor
    for i = 1:length(obj.ChargersList)
        for c = 1:obj.C
            % Loop does not run for obj.num_passenger_flows = 0
            for k = 1:obj.num_passenger_flows
                f_cost_pax(obj.FindChargeLinkPtckl(t,c,k,i)) = obj.ValueOfTime*obj.ChargerTime(i) + abs(obj.ChargerSpeed(i))*obj.BatteryDepreciationPerUnitCharge;
                f_cost_pax(obj.FindDischargeLinkPtckl(t,c,k,i)) = obj.ValueOfTime*obj.ChargerTime(i) + abs(obj.ChargerSpeed(i))*obj.BatteryDepreciationPerUnitCharge;
            end
            f_cost_reb(obj.FindChargeLinkRtcl(t,c,i))= abs(obj.ChargerSpeed(i))*obj.BatteryDepreciationPerUnitCharge;
            f_cost_reb(obj.FindDischargeLinkRtcl(t,c,i))= abs(obj.ChargerSpeed(i))*obj.BatteryDepreciationPerUnitCharge;       
        end
    end
end

% Cost for source relaxation
if obj.sourcerelaxflag
    f_cost_relax(obj.FindSourceRelaxks(1,1):obj.FindSourceRelaxks(obj.M,length(obj.Sources{obj.M}))) = obj.SourceRelaxCost;
end

f_cost = f_cost_pax + f_cost_reb + f_cost_relax;

end