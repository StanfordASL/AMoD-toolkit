function ConfigurePlot(params_plot,h)
ax = h.CurrentAxes;
ax.LineWidth = params_plot.alw;
set(ax,'TickLabelInterpreter','latex');

h.PaperUnits = 'inches';
h.PaperPosition = [0, 0, params_plot.BoxDim];

all_text_handles = findall(ax, 'Type', 'text');
set([ax; all_text_handles], 'FontName', 'Times New Roman', 'FontSize', params_plot.fsz);
set(all_text_handles,'Interpreter','latex');
end

