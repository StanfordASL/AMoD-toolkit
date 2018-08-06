function Initialize(obj)
if obj.verbose
    DispWithTimeStamp('Starting initalization of eamod_problem.')
end

obj.state_range = 1:obj.FindEndRebLocationci(obj.spec.C,obj.spec.N);
obj.relax_range = (obj.FindEndRebLocationci(obj.spec.C,obj.spec.N) + 1):obj.StateSize;

obj.decision_variables = DefineDecisionVariables(obj);

obj.is_initialized = true;

if obj.verbose
    DispWithTimeStamp('Finished initalization of eamod_problem.')
end
end