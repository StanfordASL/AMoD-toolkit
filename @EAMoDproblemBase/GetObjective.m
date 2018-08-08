function objective = GetObjective(obj)
% GetObjective Returns the optimization objective for the electric AMoD problem (excluding electricity costs)

objective = obj.GetAMoDcost();
end