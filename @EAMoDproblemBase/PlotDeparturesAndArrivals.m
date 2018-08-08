function figure_handle = PlotDeparturesAndArrivals(obj,varargin)
% PlotDeparturesAndArrivals Creates a plot showing the amount of trips departing and arriving across time.
%   figure_handle = PlotDeparturesAndArrivals(obj) uses the decision_vector in obj.decision_variables
%   figure_handle = PlotDeparturesAndArrivals(obj,decision_vector_val) uses decision_vector_val

switch numel(varargin)
    case 0
        decision_vector_val = obj.EvaluateDecisionVector();
    case 1
        decision_vector_val = varargin{1};
    otherwise
        error('Too many arguments.')
end

[DepTimeHist, ArrivalTimeHist] = obj.GetTravelTimesHistograms(decision_vector_val);

if obj.spec.time_step_s == 0
    time = 1:obj.spec.Thor;
    x_label = 'Time step';
else
    
    time = GetDateTimeRange(obj.spec);   
    x_label = 'Time [h]';
end


PassengerCarsDeltaHist = DepTimeHist - ArrivalTimeHist;

PassengerCarsHist = cumsum(PassengerCarsDeltaHist);

figure_handle = figure;
hold on
plot(time,DepTimeHist,'-.')
plot(time,ArrivalTimeHist,'--')
plot(time,PassengerCarsDeltaHist,'-')
plot(time,PassengerCarsHist,':')

legend('Departures','Arrivals','Change in passenger-carrying vehicles','Passenger-carrying vehicles','location','northeast')
xlabel(x_label)
ylabel('Number of vehicles [-]')

xlim([time(1),time(end)])
end