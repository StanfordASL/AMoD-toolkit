function state_vector_val = EvaluateStateVector(obj)
% EvaluateStateVector Returns the value of state_vector

state_vector_val = value(obj.optimization_variables.state_vector);
end