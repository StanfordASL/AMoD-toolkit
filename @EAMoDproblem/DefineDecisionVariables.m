function decision_variables = DefineDecisionVariables(obj)
% DefineDecisionVariables Defines the sdpvars used by YALMIP and returns them in the struct decision_variables

decision_variables.decision_vector = sdpvar(obj.StateSize,1);

end