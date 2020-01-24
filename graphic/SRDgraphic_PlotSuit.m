%plots positions graphs
function SRDgraphic_PlotSuit(SimulationOutput, varargin)
Parser = inputParser;
Parser.FunctionName = 'SRDgraphic_PlotPositions';
Parser.addOptional('Indeces', 1:size(SimulationOutput.Position, 2));
Parser.parse(varargin{:});

if isfield(SimulationOutput, 'MeasuredPosition')
    figure('Color', 'w', 'Name', 'Measured Position');
    subplot(2, 2, 1)
    SRDgraphic_PlotPositions(SimulationOutput, 'Indeces', Parser.Results.Indeces, 'NewFigure', false);
    subplot(2, 2, 2)
    SRDgraphic_PlotMeasuredPosition(SimulationOutput, 'Indeces', Parser.Results.Indeces, 'NewFigure', false);
    subplot(2, 2, 3:4)
    SRDgraphic_PlotPositions_vs_MeasuredPosition(SimulationOutput, 'Indeces', Parser.Results.Indeces, 'NewFigure', false);
end

if isfield(SimulationOutput, 'MeasuredVelocity')
    figure('Color', 'w', 'Name', 'Measured Velocity');
    subplot(2, 2, 1)
    SRDgraphic_PlotVelocities(SimulationOutput, 'Indeces', Parser.Results.Indeces, 'NewFigure', false);
    subplot(2, 2, 2)
    SRDgraphic_PlotMeasuredVelocity(SimulationOutput, 'Indeces', Parser.Results.Indeces, 'NewFigure', false);
    subplot(2, 2, 3:4)
    SRDgraphic_PlotVelocities_vs_MeasuredVelocity(SimulationOutput, 'Indeces', Parser.Results.Indeces, 'NewFigure', false);
end

figure('Color', 'w');
subplot(2, 2, 1)
SRDgraphic_PlotPositions(SimulationOutput, 'Indeces', Parser.Results.Indeces, 'NewFigure', false);
subplot(2, 2, 2)
SRDgraphic_PlotDesiredPosition(SimulationOutput, 'Indeces', Parser.Results.Indeces, 'NewFigure', false);
subplot(2, 2, 3)
SRDgraphic_PlotVelocities(SimulationOutput, 'Indeces', Parser.Results.Indeces, 'NewFigure', false);
subplot(2, 2, 4)
if max(Parser.Results.Indeces) > size(SimulationOutput.ControlActions, 2)
    Indeces = 1:size(SimulationOutput.ControlActions, 2);
else
    Indeces = Parser.Results.Indeces;
end
SRDgraphic_PlotControlActions(SimulationOutput, 'Indeces', Indeces, 'NewFigure', false);

end