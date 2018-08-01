% chargher_power_w = A_charger_power_w*obj.decision_variables.decision_vector;
% as in GetChargerPowerDemand. The result is a vector indexed by 
function A_charger_power_w = ComputeChargerPowerMatrixNew(obj)

n_row = obj.NumChargers*obj.Thor;

n_entry = 2*obj.Thor*obj.C*(obj.num_passenger_flows + 1)*sum(obj.ChargerTime);

A_sparse = zeros(n_entry,3);

i_row = 1;
i_entry = 1;

for t = 1:obj.Thor
    for l = 1:obj.NumChargers %Go through chargers in the road network
        charger_power_w = obj.ChargerSpeed(l)/obj.ChargerTime(l)*obj.charge_unit_j/obj.time_step_s;
        for c = 1:obj.C
            if c + obj.ChargerSpeed(l) <= obj.C
                % Loop does not run for obj.num_passenger_flows = 0
                for k = 1:obj.num_passenger_flows
                    % IF the charge link takes more than one unit time,
                    % we consider arcs that started at past times (up
                    % to t-ChargerTime+1),since they're still charging).
                    % In general, the amount of power per unit time
                    % delivered is ChargerSpeed/ChargerTime.
                    for deltat = 0:obj.ChargerTime(l) - 1
                        if t - deltat > 0 && t - deltat + obj.ChargerTime(l) <= obj.Thor
                            A_sparse(i_entry,:) = [obj.FindChargerPowertl(t,l),obj.FindChargeLinkPtckl(t-deltat,c,k,l),charger_power_w];
                            i_entry = i_entry + 1;
                        end
                    end
                end
                for deltat = 0:obj.ChargerTime(l)-1
                    if t - deltat > 0 && t - deltat + obj.ChargerTime(l) <= obj.Thor
                        A_sparse(i_entry,:) = [obj.FindChargerPowertl(t,l),obj.FindChargeLinkRtcl(t-deltat,c,l),charger_power_w];
                        i_entry = i_entry + 1;
                    end
                end
            end
        end
        for c = 1:obj.C
            if c - obj.ChargerSpeed(l) >= 1
                % Loop does not run for obj.num_passenger_flows = 0
                for k = 1:obj.num_passenger_flows
                    for deltat = 0:obj.ChargerTime(l) - 1
                        if t - deltat > 0 && t - deltat + obj.ChargerTime(l) <= obj.Thor
                        A_sparse(i_entry,:) = [obj.FindChargerPowertl(t,l),obj.FindDischargeLinkPtckl(t - deltat,c,k,l),-obj.v2g_efficiency*charger_power_w];
                        i_entry = i_entry + 1;                      
                        end
                    end
                end                
                for deltat = 0:obj.ChargerTime(l) - 1
                    if t - deltat > 0 && t - deltat + obj.ChargerTime(l) <= obj.Thor
                        A_sparse(i_entry,:) = [obj.FindChargerPowertl(t,l),obj.FindDischargeLinkRtcl(t - deltat,c,l),-obj.v2g_efficiency*charger_power_w];
                        i_entry = i_entry + 1;
                    end
                end
            end            
        end
        i_row = i_row + 1;
    end
end

assert(i_row - 1 == n_row);

if (i_entry - 1 > n_entry)
    warning('More entries than expected.')
end

% Remove extra rows in Aeqsparse
A_sparse = A_sparse(1:(i_entry - 1),:);

A_charger_power_w = sparse(A_sparse(:,1),A_sparse(:,2),A_sparse(:,3),n_row,obj.StateSize);
end