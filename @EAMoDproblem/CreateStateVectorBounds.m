function [lb_StateVector,ub_StateVector] = CreateStateVectorBounds(obj)
% CreateStateVectorBounds Creates upper and lower bounds for the state vector in the LP
%   [lb_StateVector,ub_StateVector] = CreateStateVectorBounds(obj)

lb_StateVector = zeros(obj.StateSize,1); %Passenger and rebalancing flows, passenger sources and sinks
ub_StateVector = Inf*ones(obj.StateSize,1); %Why not? We enforce capacity separately

% We have to be careful with discharging constraints: in particular, we
% should pin discharging links that do not appear in the equalities to
% zero.
% ChargeLink(t,c,..) starts at time t and goes up. So we exclude it if
% either t+charger_time>n_time_step or c+charger_speed>n_charge_step.
% DischargeLink(t,c,...) starts at time t and goes down. So we exclude it
% if either t+charger_time>n_time_step or c-charger_speed<1.
for l = 1:obj.spec.n_charger
    for c = 1:obj.spec.n_charge_step
        for t = 1:obj.spec.n_time_step
            if t + obj.spec.charger_time(l) > obj.spec.n_time_step
                %add link to i,c+1.
                % Loop does not run for obj.num_passenger_flows = 0
                for k = 1:obj.num_passenger_flows
                    ub_StateVector(obj.FindChargeLinkPtckl(t,c,k,l)) = 0;
                    ub_StateVector(obj.FindDischargeLinkPtckl(t,c,k,l)) = 0;
                end
                ub_StateVector(obj.FindChargeLinkRtcl(t,c,l)) = 0;
                ub_StateVector(obj.FindDischargeLinkRtcl(t,c,l)) = 0;
            end
            if c + obj.spec.charger_speed(l) > obj.spec.n_charge_step
                % Loop does not run for obj.num_passenger_flows = 0
                for k = 1:obj.num_passenger_flows
                    ub_StateVector(obj.FindChargeLinkPtckl(t,c,k,l)) = 0;
                end
                ub_StateVector(obj.FindChargeLinkRtcl(t,c,l)) = 0;
            end
            if c - obj.spec.charger_speed(l) < 1
                % Loop does not run for obj.num_passenger_flows = 0
                for k = 1:obj.num_passenger_flows
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
            % Loop does not run for obj.num_passenger_flows = 0
            for k = 1:obj.num_passenger_flows
                ub_StateVector(obj.FindDischargeLinkPtckl(t,c,k,l)) = 0;
            end
            ub_StateVector(obj.FindDischargeLinkRtcl(t,c,l)) = 0;
        end
        for c = (obj.spec.n_charge_step - obj.spec.charger_speed(l) + 1):obj.spec.n_charge_step
            % Loop does not run for obj.num_passenger_flows = 0
            for k = 1:obj.num_passenger_flows
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
            % Loop does not run for obj.num_passenger_flows = 0
            for k = 1:obj.num_passenger_flows
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
                for k = 1:obj.num_passenger_flows
                    ub_StateVector(obj.FindDischargeLinkPtckl(t,c,k,l)) = 0;
                end
                ub_StateVector(obj.FindDischargeLinkRtcl(t,c,l)) = 0;            
            end
        end    
    end
end

% We check obj.use_real_time_formulation because this appears in TVPowerBalancedFlow_realtime
% but not in TVPowerBalancedFlow_withpower_bundle.
if obj.use_real_time_formulation
    % Number of relaxed pax should never be negative
    if obj.sourcerelaxflag
        for k= 1:obj.spec.n_passenger_flow
            for ssi=1:length(obj.spec.passenger_source_list_cell{k})
                ub_StateVector(obj.FindSourceRelaxks(k,ssi)) = obj.spec.passenger_flow_list_cell{k}(ssi);
                lb_StateVector(obj.FindSourceRelaxks(k,ssi)) = 0;
            end
        end
    end
end


end













