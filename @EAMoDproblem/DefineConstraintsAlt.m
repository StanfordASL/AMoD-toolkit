function constraint_array = DefineConstraintsAlt(obj)
[A_eq,B_eq,A_in,B_in] = obj.CreateConstraintMatrices();

A_in_applies = isfinite(B_in);

A_in_red = A_in(A_in_applies,:);
B_in_red = B_in(A_in_applies,:);

[lb_StateVector,ub_StateVector] = obj.CreateStateVectorBounds;


lb_applies = isfinite(lb_StateVector);
ub_applies = isfinite(ub_StateVector);

decision_vector = obj.decision_variables.decision_vector;

constraint_array = [
    A_eq*decision_vector == B_eq;
    A_in_red*decision_vector <= B_in_red;
    lb_StateVector(lb_applies) <= decision_vector(lb_applies);
    decision_vector(ub_applies) <= ub_StateVector(ub_applies);
    ];
end