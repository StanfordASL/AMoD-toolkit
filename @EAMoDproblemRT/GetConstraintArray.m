function constraint_array = GetConstraintArray(obj)
% GetConstraintArray Returns the constraints for the electric AMoD problem for use in YALMIP
%
% See also AbstractEAMoDproblem.GetConstraintArray

state_vector = obj.optimization_variables.state_vector;
[Aeq_CustomerChargeConservation, Beq_CustomerChargeConservation] = obj.CreateEqualityConstraintMatrices_CustomerChargeConservation();
    
customer_charge_conservation = Aeq_CustomerChargeConservation*state_vector == Beq_CustomerChargeConservation;
customer_charge_conservation = TagConstraintIfNonEmpty(customer_charge_conservation,'CustomerChargeConservation');

constraint_array = [
    customer_charge_conservation;
    obj.GetCommonConstraintArray();
];

end