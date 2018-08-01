function h = PlotVehicleState(obj,params_plot,title_text,varargin)

switch numel(varargin)
    case 0
        decision_vector_val = obj.EvaluateDecisionVector();
    case 1
        decision_vector_val = varargin{1};
    otherwise
        error('Too many arguments.')    
end

[ChargingVehicleHist,DischargingVehicleHist,PaxVehicleHist,...
            RebVehicleHist,IdleVehicleHist,AllVehicleHist] ...
            = obj.GetVehicleStateHistograms(decision_vector_val);
        
date_time_range = GetDateTimeRange(obj.eamod_spec);

% Note that we are concatenating horizontally. Multi-line notation is only
% for convenience.
state_matrix = [ChargingVehicleHist(:),DischargingVehicleHist(:),RebVehicleHist(:),PaxVehicleHist(:),IdleVehicleHist(:)];

%legend_cell = {'Passenger-carrying vehicles','Rebalancing vehicles','Charging vehicles','Discharging vehicles','Idle vehicles'};
legend_cell = {'Charging vehicles','Discharging vehicles','Rebalancing vehicles','Passenger-carrying vehicles','Idle vehicles'};

% Do not plot Discharging vehicles if there are none of them
if all(DischargingVehicleHist < 1e-3)
 DischargingVehicleHist_ind = 2;
 state_matrix(:,DischargingVehicleHist_ind) = [];
 legend_cell(:,DischargingVehicleHist_ind) = [];    
end
    
h = figure;

h_area = area(date_time_range,state_matrix);

n_area = numel(h_area);

for i_area = 1:n_area
    h_area(i_area).FaceColor = params_plot.color_map(i_area,:);
end

legend(legend_cell,'location','northeast')
title(title_text)
xlabel('Time [h]')
ylabel('Number of vehicles [-]')

xlim([date_time_range(1),date_time_range(end)])

ConfigurePlot(params_plot,h);



end