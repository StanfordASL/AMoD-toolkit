function constraint_tagged = TagConstraintIfNonEmpty(constraint,tag)
% TagConstraintIfNonEmpty Tags a YALMIP constraint if it is not empty
%   constraint_tagged = TagConstraintIfNonEmpty(constraint,tag)
%   Note that just tagging an empty constraint leads to an error

if isempty(constraint)
    constraint_tagged = [];
else
    constraint_tagged = constraint:tag;
end
end
