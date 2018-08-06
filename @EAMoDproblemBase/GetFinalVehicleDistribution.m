function final_vehicle_distribution = GetFinalVehicleDistribution(obj,varargin)

switch numel(varargin)
    case 0
        decision_vector_val = obj.EvaluateDecisionVector();
    case 1
        decision_vector_val = varargin{1};
    otherwise
        error('Too many arguments.')
end

final_vehicle_distribution = zeros(obj.spec.C,obj.spec.N);

for c = 1:obj.spec.C
    for i = 1:obj.spec.N
        final_vehicle_distribution(c,i) = decision_vector_val(obj.FindEndRebLocationci(c,i));
    end
end

end