function DispWithTimeStamp(str)
% DispWithTimeStamp Prints the argument string together with the current timestamp

fprintf('%s: %s\n',char(datetime('now')),str);
end