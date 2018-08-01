function Initialize(obj)
DispWithTimeStamp('Starting initalization of eamod_problem.')
obj.state_range = 1:obj.FindEndRebLocationci(obj.C,obj.N);
obj.relax_range = (obj.FindEndRebLocationci(obj.C,obj.N) + 1):obj.StateSize;

obj.decision_variables = DefineDecisionVariables(obj);

obj.is_initialized = true;
DispWithTimeStamp('Finished initalization of eamod_problem.')
end