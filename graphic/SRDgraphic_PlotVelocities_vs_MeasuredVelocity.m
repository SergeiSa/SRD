%plots Position vs DesiredPosition graphs
function SRDgraphic_PlotVelocities_vs_MeasuredVelocity(SimulationOutput, varargin)
Parser = inputParser;
Parser.FunctionName = 'SRDgraphic_PlotVelocities_vs_MeasuredVelocity';
Parser.addOptional('NewFigure', true);
Parser.addOptional('Indeces', 1:size(SimulationOutput.Velocity, 2));
Parser.parse(varargin{:});

NewFigure = Parser.Results.NewFigure;

if NewFigure
    figure_handle = figure('Color', 'w', 'Name', 'Velocities vs MeasuredVelocity');
end

labels = {};
for i = 1:length(Parser.Results.Indeces)
    index = Parser.Results.Indeces(i);
    
    plot(SimulationOutput.Time, SimulationOutput.Velocity(:, index), ...
        'LineWidth', SRDgraphic_get_LineWidth(i)+1, 'LineStyle', SRDgraphic_get_LineStyle(i), ...
        'Color', SRDgraphic_FixColor(SRDgraphic_get_Color(i)-[0.15 0.15 0.15])); hold on;
    
    plot(SimulationOutput.Time, SimulationOutput.MeasuredVelocity(:, index), ...
        'LineWidth', SRDgraphic_FixLineWidth(SRDgraphic_get_LineWidth(i)-0.5), 'LineStyle', SRDgraphic_get_LineStyle(i), ...
        'Color', SRDgraphic_FixColor(SRDgraphic_get_Color(i)+[0.15 0.15 0.15]));
    
    j = 2*(i - 1) + 1;
    labels{j} = ['$$\dot q_', num2str(index), '$$'];
    labels{j+1} = ['$$\hat{\dot q}_', num2str(index), '$$'];
end

grid on; grid minor;
ax = gca;
ax.GridAlpha = 0.6;
ax.LineWidth = 0.5;
ax.MinorGridLineStyle = '-';
ax.MinorGridAlpha = 0.2;
ax.FontName = 'Times New Roman';
ax.FontSize = 18;
xlabel_handle = xlabel('$$t$$, s');
xlabel_handle.Interpreter = 'latex';
ylabel_handle = ylabel('$$\dot q_i$$');
ylabel_handle.Interpreter = 'latex';

legend_handle = legend(labels);
legend_handle.Interpreter = 'latex';
end