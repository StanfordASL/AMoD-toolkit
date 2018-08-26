function constraint_array = GetConstraintArray(obj)
% GetConstraintArray Returns the constraints for the electric AMoD problem for use in YALMIP
%
% See also AbstractEAMoDproblem.GetCommonConstraintArray

state_vector = obj.optimization_variables.state_vector;
[Aeq_PaxConservation, Beq_PaxConservation] = obj.CreateEqualityConstraintMatrices_PaxConservation();

pax_conservation = Aeq_PaxConservation*state_vector == Beq_PaxConservation;
pax_conservation = TagConstraintIfNonEmpty(pax_conservation,'PaxConservation');

constraint_array = [
    pax_conservation;
    obj.GetCommonConstraintArray();    
];

end