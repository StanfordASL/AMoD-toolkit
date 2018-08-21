function [time_range,label] = GetTimeRange(obj)
% GetTimeRange Returns the range of time-steps modelled in the spec.
%   time_range = GetDateTimeRange(obj) 
%   time_range is of type datetime if obj.time_step_s and obj.start_date_time are set
%   time_range = 1:obj.Thor otherwise

if isempty(obj.time_step_s) || isempty(obj.start_date_time)
    time_range = 1:obj.Thor;
    label = 'Time step';
else
    time_step_duration = seconds(obj.time_step_s);
    
    end_date_time =  obj.start_date_time + time_step_duration*(obj.n_time_step - 1);
    
    time_range = start_date_time:time_step_duration:end_date_time;
    
    label = 'Time [h]';
end
end