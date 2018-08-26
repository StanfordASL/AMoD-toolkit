function figure_handle = PlotVehicleState(obj,params_plot,varargin)
% PlotVehicleState Creates a plot showing the amount of vehicles in the different states (charging, discharging, rebalancing, carrying passengers or idling) across time.
%   figure_handle = PlotVehicleState(obj) uses the state_vector in obj.optimization_variables
%   figure_handle = PlotVehicleState(obj,decision_vector_val) uses decision_vector_val
%   figure_handle = PlotVehicleState(...,'color_map',color_map) uses color_map for the graph's colors

p = inputParser;
addOptional(p,'decision_vector_val',obj.EvaluateDecisionVector());
addParameter(p,'color_map',[]);
parse(p,varargin{:});

decision_vector_val = p.Results.decision_vector_val;
color_map = p.Results.color_map;


[ChargingVehicleHist,DischargingVehicleHist,PaxVehicleHist,...
    RebVehicleHist,IdleVehicleHist] ...
    = obj.GetVehicleStateHistograms(decision_vector_val);

[time_range,x_label] = obj.spec.GetTimeRange();


state_matrix = [ChargingVehicleHist(:),DischargingVehicleHist(:),RebVehicleHist(:),PaxVehicleHist(:),IdleVehicleHist(:)];

legend_cell = {'Charging vehicles','Discharging vehicles','Rebalancing vehicles','Passenger-carrying vehicles','Idle vehicles'};

% Do not plot discharging vehicles if there are none of them
if all(DischargingVehicleHist < 1e-3)
    DischargingVehicleHist_ind = 2;
    state_matrix(:,DischargingVehicleHist_ind) = [];
    legend_cell(:,DischargingVehicleHist_ind) = [];
end

figure_handle = figure;

h_area = area(time_range,state_matrix);

n_area = numel(h_area);

if ~isempty(color_map)
    for i_area = 1:n_area
        h_area(i_area).FaceColor = params_plot.color_map(i_area,:);
    end
end

legend(legend_cell,'location','northeast')
xlabel(x_label)
ylabel('Number of vehicles [-]')

xlim([time_range(1),time_range(end)])

end