function [lb_StateVector,ub_StateVector] = CreateStateVectorBounds(obj)
% CreateStateVectorBounds Creates upper and lower bounds for the state vector in the LP
%   [lb_StateVector,ub_StateVector] = CreateStateVectorBounds(obj)

lb_StateVector = zeros(obj.n_state_vector,1); % Passenger and rebalancing flows, passenger sources and sinks
ub_StateVector = Inf*ones(obj.n_state_vector,1);

for l = 1:obj.spec.n_charger
    for c = 1:obj.spec.n_charge_step
        for t = 1:obj.spec.n_time_step
            if t + obj.spec.charger_time(l) > obj.spec.n_time_step
                % add link to i,c+1.
                % Loop does not run for obj.n_passenger_flow_in_optimization = 0
                for k = 1:obj.n_passenger_flow_in_optimization
                    ub_StateVector(obj.FindChargeLinkPtckl(t,c,k,l)) = 0;
                    ub_StateVector(obj.FindDischargeLinkPtckl(t,c,k,l)) = 0;
                end
                ub_StateVector(obj.FindChargeLinkRtcl(t,c,l)) = 0;
                ub_StateVector(obj.FindDischargeLinkRtcl(t,c,l)) = 0;
            end
            if c + obj.spec.charger_speed(l) > obj.spec.n_charge_step
                % Loop does not run for obj.n_passenger_flow_in_optimization = 0
                for k = 1:obj.n_passenger_flow_in_optimization
                    ub_StateVector(obj.FindChargeLinkPtckl(t,c,k,l)) = 0;
                end
                ub_StateVector(obj.FindChargeLinkRtcl(t,c,l)) = 0;
            end
            if c - obj.spec.charger_speed(l) < 1
                % Loop does not run for obj.n_passenger_flow_in_optimization = 0
                for k = 1:obj.n_passenger_flow_in_optimization
                    ub_StateVector(obj.FindDischargeLinkPtckl(t,c,k,l)) = 0;
                end
                ub_StateVector(obj.FindDischargeLinkRtcl(t,c,l)) = 0;
            end
        end
    end
end

% Minimum end charge
for i = 1:obj.spec.n_road_node
    for c = 1:obj.spec.final_min_charge
        ub_StateVector(obj.FindEndRebLocationci(c,i)) = 0;
    end
end

% Don't create chargers/dischargers from thin air
% Prevent charging that would result in invalid charge levels i.e., outside [0,obj.spec.n_charge_step]
for t = 1:obj.spec.n_time_step
    for l = 1:obj.spec.n_charger
        for c = 1:obj.spec.charger_speed(l)
            % Loop does not run for obj.n_passenger_flow_in_optimization = 0
            for k = 1:obj.n_passenger_flow_in_optimization
                ub_StateVector(obj.FindDischargeLinkPtckl(t,c,k,l)) = 0;
            end
            ub_StateVector(obj.FindDischargeLinkRtcl(t,c,l)) = 0;
        end
        for c = (obj.spec.n_charge_step - obj.spec.charger_speed(l) + 1):obj.spec.n_charge_step
            % Loop does not run for obj.n_passenger_flow_in_optimization = 0
            for k = 1:obj.n_passenger_flow_in_optimization
                ub_StateVector(obj.FindChargeLinkPtckl(t,c,k,l)) = 0;
            end
            ub_StateVector(obj.FindChargeLinkRtcl(t,c,l)) = 0;
        end
    end
end

% Prevent charging that would end after n_time_step
for l = 1:obj.spec.n_charger
    for t = (obj.spec.n_time_step - obj.spec.charger_time(l) + 1):obj.spec.n_time_step
        for c = 1:obj.spec.n_charge_step
            % Loop does not run for obj.n_passenger_flow_in_optimization = 0
            for k = 1:obj.n_passenger_flow_in_optimization
                ub_StateVector(obj.FindChargeLinkPtckl(t,c,k,l)) = 0;
                ub_StateVector(obj.FindDischargeLinkPtckl(t,c,k,l)) = 0;
            end
            ub_StateVector(obj.FindChargeLinkRtcl(t,c,l)) = 0;
            ub_StateVector(obj.FindDischargeLinkRtcl(t,c,l)) = 0;
        end
    end
end

% If spec.disable_v2g is true, set all discharging flows to zero. 
% lb_StateVector is already set to zero, so we only need to set ub_StateVector to zero as well
if obj.spec.disable_v2g
    for l = 1:obj.spec.n_charger
        for c = 1:obj.spec.n_charge_step
            for t = 1:obj.spec.n_time_step
                for k = 1:obj.n_passenger_flow_in_optimization
                    ub_StateVector(obj.FindDischargeLinkPtckl(t,c,k,l)) = 0;
                end
                ub_StateVector(obj.FindDischargeLinkRtcl(t,c,l)) = 0;            
            end
        end    
    end
end

end













