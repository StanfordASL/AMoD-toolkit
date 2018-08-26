function constraint_array = GetCommonConstraintArray(obj)
% GetCommonConstraintArray Returns the constraints for the electric AMoD problem for use in YALMIP that appear in both the standard and real-time formulations

if obj.verbose
    DispWithTimeStamp('Starting GetConstraintArray of eamod_problem.');
end

state_vector = obj.optimization_variables.state_vector;

[Aeq_RebConservation, Beq_RebConservation] = obj.CreateEqualityConstraintMatrices_RebConservation();

reb_conservation = Aeq_RebConservation*state_vector == Beq_RebConservation;
reb_conservation = TagConstraintIfNonEmpty(reb_conservation,'RebConservation');

[Aeq_SourceConservation, Beq_SourceConservation] = obj.CreateEqualityConstraintMatrices_SourceConservation();

source_conservation = Aeq_SourceConservation*state_vector == Beq_SourceConservation;
source_conservation = TagConstraintIfNonEmpty(source_conservation,'SourceConservation');

[Aeq_SinkConservation, Beq_SinkConservation] = obj.CreateEqualityConstraintMatrices_SinkConservation();

sink_conservation = Aeq_SinkConservation*state_vector == Beq_SinkConservation;
sink_conservation = TagConstraintIfNonEmpty(sink_conservation,'SinkConservation');

[Ain_RoadCongestion, Bin_RoadCongestion] = obj.CreateInequalityConstraintMatrices_RoadCongestion();

% Apply constraint only for finite bounds
road_congestion_indices = isfinite(Bin_RoadCongestion);
Ain_RoadCongestion_red = Ain_RoadCongestion(road_congestion_indices,:);
Bin_RoadCongestion_red = Bin_RoadCongestion(road_congestion_indices,:);

road_congestion_bound = Ain_RoadCongestion_red*state_vector <= Bin_RoadCongestion_red;
road_congestion_bound = TagConstraintIfNonEmpty(road_congestion_bound,'RoadCongestion');

[Ain_ChargerCongestion, Bin_ChargerCongestion] = obj.CreateInequalityConstraintMatrices_ChargerCongestion();

charger_congestion_indices = isfinite(Bin_ChargerCongestion);
Ain_ChargerCongestion_red = Ain_ChargerCongestion(charger_congestion_indices,:);
Bin_ChargerCongestion_red = Bin_ChargerCongestion(charger_congestion_indices,:);

charger_congestion_bound = Ain_ChargerCongestion_red*state_vector <= Bin_ChargerCongestion_red;
charger_congestion_bound = TagConstraintIfNonEmpty(charger_congestion_bound,'ChargerCongestion');

[lb_StateVector,ub_StateVector] = obj.CreateStateVectorBounds();

decision_vector_lower_bound_indices = isfinite(lb_StateVector);
decision_vector_lower_bound = lb_StateVector(decision_vector_lower_bound_indices) <= state_vector(decision_vector_lower_bound_indices);
decision_vector_lower_bound = TagConstraintIfNonEmpty(decision_vector_lower_bound,'StateVectorLowerBound');

decision_vector_upper_bound_indices = isfinite(ub_StateVector);
decision_vector_upper_bound =  state_vector(decision_vector_upper_bound_indices) <= ub_StateVector(decision_vector_upper_bound_indices);
decision_vector_upper_bound = TagConstraintIfNonEmpty(decision_vector_upper_bound,'StateVectorUpperBound');

constraint_array = [    
    reb_conservation;
    source_conservation;
    sink_conservation;
    road_congestion_bound;
    charger_congestion_bound;
    decision_vector_lower_bound;
    decision_vector_upper_bound;
    ];

if obj.verbose
    DispWithTimeStamp('Finished GetConstraintArray of eamod_problem.');
end
end