function [DepTimeHist, ArrivalTimeHist] = GetTravelTimesHistograms(obj,varargin)
% GetTravelTimesHistograms Returns the number of departing and arriving trips as a function of time
%   [DepTimeHist, ArrivalTimeHist] = GetTravelTimesHistograms(obj) uses the decision_vector in obj.decision_variables
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

DepTimeHist = zeros(1,obj.spec.Thor);

for tt = 1:obj.spec.Thor
    for ll = 1:length(obj.spec.Flows)
        for ssi = 1:length(obj.spec.Flows{ll})
            if obj.spec.StartTimes{ll}(ssi) == tt
                DepTimeHist(tt) = DepTimeHist(tt) + obj.spec.Flows{ll}(ssi);
            end
        end
    end
end

% Arrival times
ArrivalTimeHist = zeros(1,obj.spec.Thor);
for tt = 1:obj.spec.Thor
    for c = 1:obj.spec.C
        for k = 1:length(obj.spec.Sinks)
            ArrivalTimeHist(tt) = ArrivalTimeHist(tt) + decision_vector_val(obj.FindPaxSinkChargetck(tt,c,k));
        end
    end
end

end