function date_time_range = GetDateTimeRange(spec)

start_date_time = spec.start_date_time;
time_step_s =  spec.time_step_s;
time_step_duration = seconds(time_step_s);
n_time_step =  spec.n_time_step;

end_date_time =  start_date_time + time_step_duration*(n_time_step - 1);

date_time_range = start_date_time:time_step_duration:end_date_time;
end