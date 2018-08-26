function [lb_StateVector,ub_StateVector] = CreateStateVectorBounds(obj)
% CreateStateVectorBounds Creates upper and lower bounds for the state vector in the LP
%   [lb_StateVector,ub_StateVector] = CreateStateVectorBounds(obj)
%
%   See also AbstractEAMoDproblem.CreateStateVectorBounds 

% We overide because, in AMoD-power, this appears in TVPowerBalancedFlow_realtime
% but not in TVPowerBalancedFlow_withpower_bundle.

[lb_StateVector,ub_StateVector] = CreateStateVectorBounds@AbstractEAMoDproblem(obj);

% Number of relaxed pax should never be negative
if obj.source_relax_flag
    for k= 1:obj.spec.n_passenger_flow
        for ssi=1:length(obj.spec.passenger_source_list_cell{k})
            ub_StateVector(obj.FindSourceRelaxks(k,ssi)) = obj.spec.passenger_flow_list_cell{k}(ssi);
            lb_StateVector(obj.FindSourceRelaxks(k,ssi)) = 0;
        end
    end
end

end