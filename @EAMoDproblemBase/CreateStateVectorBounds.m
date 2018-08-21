function [lb_StateVector,ub_StateVector] = CreateStateVectorBounds(obj)
% CreateStateVectorBounds Creates upper and lower bounds for the state vector in the LP
%   [lb_StateVector,ub_StateVector] = CreateStateVectorBounds(obj)

lb_StateVector = zeros(obj.StateSize,1); %Passenger and rebalancing flows, passenger sources and sinks
ub_StateVector = Inf*ones(obj.StateSize,1); %Why not? We enforce capacity separately

% We have to be careful with discharging constraints: in particular, we
% should pin discharging links that do not appear in the equalities to
% zero.
% ChargeLink(t,c,..) starts at time t and goes up. So we exclude it if
% either t+ChargerTime>Thor or c+ChargerSpeed>C.
% DischargeLink(t,c,...) starts at time t and goes down. So we exclude it
% if either t+ChargerTime>Thor or c-ChargerSpeed<1.
for l = 1:obj.spec.NumChargers
    for c = 1:obj.spec.C
        for t = 1:obj.spec.Thor
            if t + obj.spec.ChargerTime(l) > obj.spec.Thor
                %add link to i,c+1.
                % Loop does not run for obj.num_passenger_flows = 0
                for k = 1:obj.num_passenger_flows
                    ub_StateVector(obj.FindChargeLinkPtckl(t,c,k,l)) = 0;
                    ub_StateVector(obj.FindDischargeLinkPtckl(t,c,k,l)) = 0;
                end
                ub_StateVector(obj.FindChargeLinkRtcl(t,c,l)) = 0;
                ub_StateVector(obj.FindDischargeLinkRtcl(t,c,l)) = 0;
            end
            if c + obj.spec.ChargerSpeed(l) > obj.spec.C
                % Loop does not run for obj.num_passenger_flows = 0
                for k = 1:obj.num_passenger_flows
                    ub_StateVector(obj.FindChargeLinkPtckl(t,c,k,l)) = 0;
                end
                ub_StateVector(obj.FindChargeLinkRtcl(t,c,l)) = 0;
            end
            if c - obj.spec.ChargerSpeed(l) < 1
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
for i = 1:obj.spec.N
    for c = 1:obj.spec.MinEndCharge
        ub_StateVector(obj.FindEndRebLocationci(c,i)) = 0;
    end
end

% Don't create chargers/dischargers from thin air
% Prevent charging that would result in invalid charge levels i.e., outside [0,obj.spec.C]
for t = 1:obj.spec.Thor
    for l = 1:obj.spec.NumChargers
        for c = 1:obj.spec.ChargerSpeed(l)
            % Loop does not run for obj.num_passenger_flows = 0
            for k = 1:obj.num_passenger_flows
                ub_StateVector(obj.FindDischargeLinkPtckl(t,c,k,l)) = 0;
            end
            ub_StateVector(obj.FindDischargeLinkRtcl(t,c,l)) = 0;
        end
        for c = (obj.spec.C - obj.spec.ChargerSpeed(l) + 1):obj.spec.C
            % Loop does not run for obj.num_passenger_flows = 0
            for k = 1:obj.num_passenger_flows
                ub_StateVector(obj.FindChargeLinkPtckl(t,c,k,l)) = 0;
            end
            ub_StateVector(obj.FindChargeLinkRtcl(t,c,l)) = 0;
        end
    end
end

% Prevent charging that would end after Thor
for l = 1:obj.spec.NumChargers
    for t = (obj.spec.Thor - obj.spec.ChargerTime(l) + 1):obj.spec.Thor
        for c = 1:obj.spec.C
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

% We check obj.use_real_time_formulation because this appears in TVPowerBalancedFlow_realtime 
% but not in TVPowerBalancedFlow_withpower_bundle.
if obj.use_real_time_formulation    
   % Number of relaxed pax should never be negative
    if obj.spec.sourcerelaxflag
        for k= 1:obj.spec.M
            for ssi=1:length(obj.spec.Sources{k})
                ub_StateVector(obj.FindSourceRelaxks(k,ssi)) = obj.spec.Flows{k}(ssi);
                lb_StateVector(obj.FindSourceRelaxks(k,ssi)) = 0;
            end
        end
    end
end


end













