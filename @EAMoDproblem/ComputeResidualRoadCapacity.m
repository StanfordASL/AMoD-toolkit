function road_residual_capacity_matrix = ComputeResidualRoadCapacity(obj)
% ComputeResidualRoadCapacity Computes the residual road capacity matrix. In the standard formulation it matches spec.road_capacity_matrix since there are no pre-routed vehicles

road_residual_capacity_matrix = repmat(full(obj.spec.road_capacity_matrix),1,1,obj.spec.n_time_step);
end