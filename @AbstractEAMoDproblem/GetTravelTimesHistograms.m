function [DepTimeHist, ArrivalTimeHist] = GetTravelTimesHistograms(obj,varargin)
% GetTravelTimesHistograms Returns the number of departing and arriving trips as a function of time
%   [DepTimeHist, ArrivalTimeHist] = GetTravelTimesHistograms(obj) uses the state_vector in obj.optimization_variables
%   [DepTimeHist, ArrivalTimeHist] = GetTravelTimesHistograms(obj,decision_vector_val) uses decision_vector_val
%   DepTimeHist contains the number of departing trips
%   ArrivalTimeHist contains the number of arriving trips

switch numel(varargin)
    case 0
        decision_vector_val = obj.EvaluateDecisionVector();
    case 1
        decision_vector_val = varargin{1};
    otherwise
        error('Too many arguments.')
end

DepTimeHist = zeros(1,obj.spec.n_time_step);

for tt = 1:obj.spec.n_time_step
    for ll = 1:length(obj.spec.passenger_flow_list_cell)
        for ssi = 1:length(obj.spec.passenger_flow_list_cell{ll})
            if obj.spec.passenger_start_time_list_cell{ll}(ssi) == tt
                DepTimeHist(tt) = DepTimeHist(tt) + obj.spec.passenger_flow_list_cell{ll}(ssi);
            end
        end
    end
end

% Arrival times
ArrivalTimeHist = zeros(1,obj.spec.n_time_step);
for tt = 1:obj.spec.n_time_step
    for c = 1:obj.spec.n_charge_step
        for k = 1:length(obj.spec.passenger_sink_list)
            ArrivalTimeHist(tt) = ArrivalTimeHist(tt) + decision_vector_val(obj.FindPaxSinkChargetck(tt,c,k));
        end
    end
end

end