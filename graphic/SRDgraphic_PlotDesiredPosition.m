%plots desired positions graphs
function SRDgraphic_PlotDesiredPosition(SimulationOutput, varargin)
Parser = inputParser;
Parser.FunctionName = 'SRDgraphic_PlotDesiredPosition';
Parser.addOptional('NewFigure', true);
Parser.addOptional('Indeces', 1:size(SimulationOutput.DesiredPosition, 2));
Parser.parse(varargin{:});

NewFigure = Parser.Results.NewFigure;

if NewFigure
    figure_handle = figure('Color', 'w', 'Name', 'Desired Positions');
end

labels = {};
for i = 1:length(Parser.Results.Indeces)
    index = Parser.Results.Indeces(i);
    
    plot(SimulationOutput.Time, SimulationOutput.DesiredPosition(:, index), ...
        'LineWidth', SRDgraphic_get_LineWidth(i), 'LineStyle', SRDgraphic_get_LineStyle(i), ...
        'Color', SRDgraphic_get_Color(i)); hold on;
    
    labels{i} = ['$$q^*_', num2str(index), '$$'];
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
ylabel_handle = ylabel('$$q^*_i$$');
ylabel_handle.Interpreter = 'latex';

legend_handle = legend(labels);
legend_handle.Interpreter = 'latex';
end