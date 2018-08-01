function [DepTimeHist, ArrivalTimeHist] = GetTravelTimesHistograms(obj,varargin)
switch numel(varargin)
    case 0
        decision_vector_val = obj.EvaluateDecisionVector();
    case 1
        decision_vector_val = varargin{1};
    otherwise
        error('Too many arguments.')    
end

DepTimeHist = zeros(1,obj.Thor);

for tt = 1:obj.Thor
    for ll = 1:length(obj.Flows)
        for ssi = 1:length(obj.Flows{ll})
            if obj.StartTimes{ll}(ssi) == tt
                DepTimeHist(tt) = DepTimeHist(tt) + obj.Flows{ll}(ssi);
            end
        end
    end
end

% Arrival times
ArrivalTimeHist = zeros(1,obj.Thor);
for tt = 1:obj.Thor
    for c = 1:obj.C
        for k = 1:length(obj.Sinks)
            ArrivalTimeHist(tt) = ArrivalTimeHist(tt) + decision_vector_val(obj.FindPaxSinkChargetck(tt,c,k));
        end
    end
end

end