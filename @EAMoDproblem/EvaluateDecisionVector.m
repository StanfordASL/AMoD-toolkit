function decision_vector_val = EvaluateDecisionVector(obj)
% EvaluateDecisionVector Returns the value of state_vector

decision_vector_val = value(obj.optimization_variables.state_vector);
end