function ConfigurePlot(params_plot,fig_handle)
% ConfigurePlot Configures the appearance of a plot according to params_plot
%   ConfigurePlot(params_plot,fig_handle) where params_plot contains the settings
%   and fig_handle is the handle to the figure.
%
%   See also GetParamsPlot

ax = fig_handle.CurrentAxes;
ax.LineWidth = params_plot.alw;
set(ax,'TickLabelInterpreter','latex');

fig_handle.PaperUnits = 'inches';
fig_handle.PaperPosition = [0, 0, params_plot.BoxDim];

all_text_handles = findall(ax, 'Type', 'text');
set([ax; all_text_handles], 'FontName', 'Times New Roman', 'FontSize', params_plot.fsz);
set(all_text_handles,'Interpreter','latex');
end

