function decision_variables = DefineDecisionVariables(obj)
    decision_variables.decision_vector = sdpvar(obj.StateSize,1);
    
    decision_variables.state_vector = decision_variables.decision_vector(obj.state_range);
    decision_variables.relax_vector = decision_variables.decision_vector(obj.relax_range);
end