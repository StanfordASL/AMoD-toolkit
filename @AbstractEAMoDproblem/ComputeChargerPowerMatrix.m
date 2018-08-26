function A_charger_power_w = ComputeChargerPowerMatrix(obj)
% ComputeChargerPowerMatrix Computes A_charger_power_w such that charger_power_w = A_charger_power_w*obj.optimization_variables.state_vector

n_row = obj.spec.n_charger*obj.spec.n_time_step;

n_entry = 2*obj.spec.n_time_step*obj.spec.n_charge_step*(obj.n_passenger_flow_in_optimization + 1)*sum(obj.spec.charger_time);

A_sparse = zeros(n_entry,3);

i_row = 1;
i_entry = 1;

for t = 1:obj.spec.n_time_step
    for l = 1:obj.spec.n_charger %Go through chargers in the road network
        charger_power_w = obj.spec.charger_speed(l)/obj.spec.charger_time(l)*obj.spec.charge_step_j/obj.spec.time_step_s;
        for c = 1:obj.spec.n_charge_step
            if c + obj.spec.charger_speed(l) <= obj.spec.n_charge_step
                % Loop does not run for obj.n_passenger_flow_in_optimization = 0
                for k = 1:obj.n_passenger_flow_in_optimization
                    % IF the charge link takes more than one unit time,
                    % we consider arcs that started at past times (up
                    % to t-charger_time+1),since they're still charging).
                    % In general, the amount of power per unit time
                    % delivered is charger_speed/charger_time.
                    for deltat = 0:obj.spec.charger_time(l) - 1
                        if t - deltat > 0 && t - deltat + obj.spec.charger_time(l) <= obj.spec.n_time_step
                            A_sparse(i_entry,:) = [obj.FindChargerPowertl(t,l),obj.FindChargeLinkPtckl(t-deltat,c,k,l),charger_power_w];
                            i_entry = i_entry + 1;
                        end
                    end
                end
                for deltat = 0:obj.spec.charger_time(l)-1
                    if t - deltat > 0 && t - deltat + obj.spec.charger_time(l) <= obj.spec.n_time_step
                        A_sparse(i_entry,:) = [obj.FindChargerPowertl(t,l),obj.FindChargeLinkRtcl(t-deltat,c,l),charger_power_w];
                        i_entry = i_entry + 1;
                    end
                end
            end
        end
        for c = 1:obj.spec.n_charge_step
            if c - obj.spec.charger_speed(l) >= 1
                % Loop does not run for obj.n_passenger_flow_in_optimization = 0
                for k = 1:obj.n_passenger_flow_in_optimization
                    for deltat = 0:obj.spec.charger_time(l) - 1
                        if t - deltat > 0 && t - deltat + obj.spec.charger_time(l) <= obj.spec.n_time_step
                            A_sparse(i_entry,:) = [obj.FindChargerPowertl(t,l),obj.FindDischargeLinkPtckl(t - deltat,c,k,l),-obj.spec.v2g_efficiency*charger_power_w];
                            i_entry = i_entry + 1;
                        end
                    end
                end
                for deltat = 0:obj.spec.charger_time(l) - 1
                    if t - deltat > 0 && t - deltat + obj.spec.charger_time(l) <= obj.spec.n_time_step
                        A_sparse(i_entry,:) = [obj.FindChargerPowertl(t,l),obj.FindDischargeLinkRtcl(t - deltat,c,l),-obj.spec.v2g_efficiency*charger_power_w];
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

A_charger_power_w = sparse(A_sparse(:,1),A_sparse(:,2),A_sparse(:,3),n_row,obj.n_state_vector);
end