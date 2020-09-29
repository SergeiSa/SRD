%plots positions graphs
function SRDgraphic_PlotGeneric(X, Y, varargin)
Parser = inputParser;
Parser.FunctionName = 'SRDgraphic_PlotGeneric';
Parser.addOptional('NewFigure', true);
Parser.addOptional('FigureName', 'Generic');
Parser.addOptional('LableVariable', 'q');
Parser.addOptional('Title', []);
Parser.parse(varargin{:});

NewFigure = Parser.Results.NewFigure;

if NewFigure
    figure('Color', 'w', 'Name', Parser.Results.FigureName);
end

N = size(Y, 2);

labels = cell(N, 1);
for i = 1:N
    plot(X, Y(:, i), ...
        'LineWidth', SRDgraphic_get_LineWidth(i), 'LineStyle', SRDgraphic_get_LineStyle(i), ...
        'Color', SRDgraphic_get_Color(i)); hold on;
    
    labels{i} = ['$$', Parser.Results.LableVariable, '_', num2str(i), '$$'];
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
ylabel_handle = ylabel(['$$', Parser.Results.LableVariable, '_i$$']);
ylabel_handle.Interpreter = 'latex';

legend_handle = legend(labels);
legend_handle.Interpreter = 'latex';

title(Parser.Results.Title);
end