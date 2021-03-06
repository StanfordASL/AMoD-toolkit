function figure_handle = PlotDeparturesAndArrivals(obj,varargin)
% PlotDeparturesAndArrivals Creates a plot showing the amount of trips departing and arriving across time.
%   figure_handle = PlotDeparturesAndArrivals(obj) uses the state_vector in obj.optimization_variables
%   figure_handle = PlotDeparturesAndArrivals(obj,state_vector_val) uses state_vector_val

switch numel(varargin)
    case 0
        state_vector_val = obj.EvaluateStateVector();
    case 1
        state_vector_val = varargin{1};
    otherwise
        error('Too many arguments.')
end

[DepTimeHist, ArrivalTimeHist] = obj.GetTravelTimesHistograms(state_vector_val);

[time_range,x_label] = obj.spec.GetTimeRange();

PassengerCarsDeltaHist = DepTimeHist - ArrivalTimeHist;

PassengerCarsHist = cumsum(PassengerCarsDeltaHist);

figure_handle = figure;
hold on
plot(time_range,DepTimeHist)
plot(time_range,ArrivalTimeHist)
plot(time_range,PassengerCarsDeltaHist,'--')
plot(time_range,PassengerCarsHist,'--')

legend('Departures','Arrivals','Change in passenger-carrying vehicles','Passenger-carrying vehicles','location','northeast')
xlabel(x_label)
ylabel('Number of vehicles [-]')

xlim([time_range(1),time_range(end)])
end