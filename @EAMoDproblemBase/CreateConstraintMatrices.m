function [A_eq,B_eq,A_in,B_in] = CreateConstraintMatrices(obj)
% CreateConstraintMatrices Consolidates constraints into matrices
%   [A_eq,B_eq,A_in,B_in] = CreateConstraintMatrices(obj) where A_eq*x =
%   B_eq and A_in*x <= B_in.
[Aeq_PaxConservation, Beq_PaxConservation] = obj.CreateEqualityConstraintMatrices_PaxConservation();
[Aeq_RebConservation, Beq_RebConservation] = obj.CreateEqualityConstraintMatrices_RebConservation;
[Aeq_SourceConservation, Beq_SourceConservation] = obj.CreateEqualityConstraintMatrices_SourceConservation;
[Aeq_SinkConservation, Beq_SinkConservation] = obj.CreateEqualityConstraintMatrices_SinkConservation;

A_eq = [
    Aeq_PaxConservation;
    Aeq_RebConservation;
    Aeq_SourceConservation;
    Aeq_SinkConservation;
    ];

B_eq = [
    Beq_PaxConservation;
    Beq_RebConservation;
    Beq_SourceConservation;
    Beq_SinkConservation;
    ];

[Ain_RoadCongestion, Bin_RoadCongestion] = obj.CreateInequalityConstraintMatrices_RoadCongestion;
[Ain_ChargerCongestion, Bin_ChargerCongestion] = obj.CreateInequalityConstraintMatrices_ChargerCongestion;

A_in = [
    Ain_RoadCongestion;
    Ain_ChargerCongestion;
    ];

B_in = [
    Bin_RoadCongestion;
    Bin_ChargerCongestion;
    ];

end