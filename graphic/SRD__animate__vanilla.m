function SRD__animate__vanilla(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD__animate__vanilla';
Parser.addOptional('TimeLog', []);
Parser.addOptional('PositionLog', []);
Parser.addOptional('AnimationTimeLog', []);

Parser.addOptional('Type', 'Default'); %'Default', 'STL', 'Custom'
Parser.addOptional('DrawRobot_function', []);
Parser.addOptional('NewFigure', true);
Parser.addOptional('FigureName', 'Animation');
Parser.addOptional('FileName_visuals_config', []);

Parser.parse(varargin{:});

SRD = SRDinterface();

if isempty(Parser.Results.FileName_visuals_config)
    FileName_visuals_config = SRD.FileName_visuals_config;
else
    FileName_visuals_config = Parser.Results.FileName_visuals_config;
end
    
visuals_config = load(FileName_visuals_config);
visuals_config = visuals_config.visuals_config;


NewFigure = Parser.Results.NewFigure;
if NewFigure
    figure('Color', 'w', 'Name', Parser.Results.FigureName);
end

view(visuals_config.ViewAngle);
axis(visuals_config.AxisLimits);

if visuals_config.Animation_ToUseGrid
    grid on;
end
if visuals_config.Animation_ToUseGridMinor
    grid minor;
end

switch Parser.Results.Type
    case 'Default'
    case 'STL'
        if ~isempty(visuals_config.DrawRobot_STL_camlight)
            camlight(visuals_config.DrawRobot_STL_camlight);
        end
        if ~isempty(visuals_config.DrawRobot_STL_material)
            material(visuals_config.DrawRobot_STL_material);
        end
end
h = [];

for i = 1:length(Parser.Results.AnimationTimeLog)
    
    target_time = Parser.Results.AnimationTimeLog(i);
    
    [~, index] = min(abs(Parser.Results.TimeLog - target_time));
    
    q = Parser.Results.PositionLog(index, :)';
    
    [az, el] = view; %remember the user-set view
    
    h = Parser.Results.DrawRobot_function(q, h); %draw new frame
    view([az, el]); %restore the user-set view
    axis equal;
    axis(visuals_config.AxisLimits);
    
    drawnow;
    if (i + 1) < length(Parser.Results.AnimationTimeLog)
        pause(Parser.Results.AnimationTimeLog(i+1) - Parser.Results.AnimationTimeLog(i));
    end
end


end