function optimization_variables = DefineDecisionVariables(obj)
% DefineDecisionVariables Defines the sdpvars used by YALMIP and returns them in the struct optimization_variables

optimization_variables.state_vector = sdpvar(obj.n_state_vector,1);

end