function road_residual_capacity_matrix = ComputeResidualRoadCapacity(obj)
% ComputeResidualRoadCapacity Computes the residual road capacity matrix. It matches spec.road_capacity_matrix since there are no pre-routed vehicles in the standard formulation.

road_residual_capacity_matrix = repmat(full(obj.spec.road_capacity_matrix),1,1,obj.spec.n_time_step);
end