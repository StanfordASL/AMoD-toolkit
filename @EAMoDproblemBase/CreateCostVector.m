function [f_cost,f_cost_pax,f_cost_reb,f_cost_relax] = CreateCostVector(obj)
f_cost_pax = zeros(obj.StateSize,1);
f_cost_reb = zeros(obj.StateSize,1);
f_cost_relax = zeros(obj.StateSize,1);

% Passengers' and rebalancers' travel time and battery depreciation
for t = 1:obj.spec.Thor
    for i = 1:obj.spec.N
        for j = obj.spec.RoadGraph{i}
            for c = 1:obj.spec.C
                % Loop does not run for obj.num_passenger_flows = 0
                for k = 1:obj.num_passenger_flows
                    % Recall that TravelDistance is in meter
                    f_cost_pax(obj.FindRoadLinkPtckij(t,c,k,i,j)) = obj.spec.ValueOfTime*obj.spec.TravelTimes(i,j) + obj.spec.TravelDistance(i,j)*obj.VehicleCostPerM + abs(obj.spec.ChargeToTraverse(i,j))*obj.BatteryDepreciationPerUnitCharge;
                end
                if j ~= i
                    % Recall that TravelDistance is in meter
                    f_cost_reb(obj.FindRoadLinkRtcij(t,c,i,j)) = obj.spec.TravelDistance(i,j)*obj.VehicleCostPerM + abs(obj.spec.ChargeToTraverse(i,j))*obj.BatteryDepreciationPerUnitCharge;
                end
            end
        end
    end
end

% Passengers' and rebalancers' charging time and cost  plus battery depreciation
for t = 1:obj.spec.Thor
    for i = 1:length(obj.spec.ChargersList)
        for c = 1:obj.spec.C
            % Loop does not run for obj.num_passenger_flows = 0
            for k = 1:obj.num_passenger_flows
                f_cost_pax(obj.FindChargeLinkPtckl(t,c,k,i)) = obj.spec.ValueOfTime*obj.spec.ChargerTime(i) + abs(obj.spec.ChargerSpeed(i))*obj.BatteryDepreciationPerUnitCharge;
                f_cost_pax(obj.FindDischargeLinkPtckl(t,c,k,i)) = obj.spec.ValueOfTime*obj.spec.ChargerTime(i) + abs(obj.spec.ChargerSpeed(i))*obj.BatteryDepreciationPerUnitCharge;
            end
            f_cost_reb(obj.FindChargeLinkRtcl(t,c,i))= abs(obj.spec.ChargerSpeed(i))*obj.BatteryDepreciationPerUnitCharge;
            f_cost_reb(obj.FindDischargeLinkRtcl(t,c,i))= abs(obj.spec.ChargerSpeed(i))*obj.BatteryDepreciationPerUnitCharge;       
        end
    end
end

% Cost for source relaxation
if obj.spec.sourcerelaxflag
    f_cost_relax(obj.FindSourceRelaxks(1,1):obj.FindSourceRelaxks(obj.spec.M,length(obj.spec.Sources{obj.spec.M}))) = obj.SourceRelaxCost;
end

f_cost = f_cost_pax + f_cost_reb + f_cost_relax;

end