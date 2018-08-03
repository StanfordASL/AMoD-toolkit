% chargher_power_w = A_charger_power_w*obj.decision_variables.decision_vector;
% as in GetChargerPowerDemand. The result is a vector indexed by 
function [A_charger_power_w] = ComputeChargerPowerMatrix(obj)

A_charger_power_w = zeros(obj.spec.NumChargers*obj.spec.Thor,obj.StateSize);

for t = 1:obj.spec.Thor
    for l = 1:obj.spec.NumChargers %Go through chargers in the road network
        for c = 1:obj.spec.C
            if c + obj.spec.ChargerSpeed(l) <= obj.spec.C
                % Loop does not run for obj.num_passenger_flows = 0
                for k = 1:obj.num_passenger_flows
                    % IF the charge link takes more than one unit time,
                    % we consider arcs that started at past times (up
                    % to t-ChargerTime+1),since they're still charging).
                    % In general, the amount of power per unit time
                    % delivered is ChargerSpeed/ChargerTime.
                    for deltat = 0:obj.spec.ChargerTime(l) - 1
                        if t - deltat > 0 && t - deltat + obj.spec.ChargerTime(l) <= obj.spec.Thor
                            A_charger_power_w(obj.FindChargerPowertl(t,l),obj.FindChargeLinkPtckl(t-deltat,c,k,l)) = obj.spec.ChargerSpeed(l)/obj.spec.ChargerTime(l)*obj.spec.charge_unit_j/obj.spec.time_step_s;
                        end
                    end
                end
                for deltat = 0:obj.spec.ChargerTime(l)-1
                    if t - deltat > 0 && t - deltat + obj.spec.ChargerTime(l) <= obj.spec.Thor
                        A_charger_power_w(obj.FindChargerPowertl(t,l),obj.FindChargeLinkRtcl(t-deltat,c,l)) = obj.spec.ChargerSpeed(l)/obj.spec.ChargerTime(l)*obj.spec.charge_unit_j/obj.spec.time_step_s;
                    end
                end
            end
        end
        for c = 1:obj.spec.C
            if c - obj.spec.ChargerSpeed(l) >= 1
                % Loop does not run for obj.num_passenger_flows = 0
                for k = 1:obj.num_passenger_flows
                    for deltat = 0:obj.spec.ChargerTime(l) - 1
                        if t - deltat > 0 && t - deltat + obj.spec.ChargerTime(l) <= obj.spec.Thor
                            A_charger_power_w(obj.FindChargerPowertl(t,l),obj.FindDischargeLinkPtckl(t - deltat,c,k,l)) = -obj.spec.v2g_efficiency*obj.spec.ChargerSpeed(l)/obj.spec.ChargerTime(l)*obj.spec.charge_unit_j/obj.spec.time_step_s;
                        end
                    end
                end
                
                for deltat = 0:obj.spec.ChargerTime(l) - 1
                    if t - deltat > 0 && t - deltat + obj.spec.ChargerTime(l) <= obj.spec.Thor
                        A_charger_power_w(obj.FindChargerPowertl(t,l),obj.FindDischargeLinkRtcl(t - deltat,c,l)) = -obj.spec.v2g_efficiency*obj.spec.ChargerSpeed(l)/obj.spec.ChargerTime(l)*obj.spec.charge_unit_j/obj.spec.time_step_s;
                    end
                end
            end            
        end
    end
end

end