function h = PlotTravelTimes(obj,params_plot,title_text,varargin)

switch numel(varargin)
    case 0
        decision_vector_val = obj.spec.EvaluateDecisionVector();
    case 1
        decision_vector_val = varargin{1};
    otherwise
        error('Too many arguments.')    
end

[DepTimeHist, ArrivalTimeHist] = obj.GetTravelTimesHistograms(decision_vector_val);
        
date_time_range = GetDateTimeRange(obj.spec);

PassengerCarsDeltaHist = DepTimeHist - ArrivalTimeHist;

PassengerCarsHist = cumsum(PassengerCarsDeltaHist);
    
h = figure;
hold on
plot(date_time_range,DepTimeHist,'-.')
plot(date_time_range,ArrivalTimeHist,'--')
plot(date_time_range,PassengerCarsDeltaHist,'-')
plot(date_time_range,PassengerCarsHist,'.')

legend('Departures','Arrivals','Change in passenger-carrying vehicles','location','northeast')
title(title_text)
xlabel('Time [h]')
ylabel('Number of vehicles [-]')

xlim([date_time_range(1),date_time_range(end)])

ConfigurePlot(params_plot,h);

end