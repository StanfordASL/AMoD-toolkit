function constraint_array = GetConstraintArray(obj)
DispWithTimeStamp('Starting GetConstraintArray of eamod_problem.');
constraint_array = obj.DefineConstraints();
DispWithTimeStamp('Finished GetConstraintArray of eamod_problem.');
end