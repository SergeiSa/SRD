function DrawRobot_function = SRD_DrawRobot_get_function(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_get_DrawRobot_function';
Parser.addOptional('DrawRobot_Type', 'Default'); %'Default' or 'STL' or 'Custom'
Parser.addOptional('DrawRobot_Custom_handle', []); 
Parser.addOptional('Function_Type', 'DrawGivenPosition'); %'DrawGivenPosition' or 'DrawInitialPosition'  or 'DrawCurrentPosition'
Parser.addOptional('FileName_visuals_config', []); 
Parser.addOptional('SimulationEngine', []);

Parser.parse(varargin{:});

SRD = SRDinterface();
    
if isempty(Parser.Results.SimulationEngine)
    SimulationEngine = SRD.GetSimulationEngine();
else
    SimulationEngine = Parser.Results.SimulationEngine;
end

if isempty(Parser.Results.FileName_visuals_config)
    FileName_visuals_config = SRD.FileName_visuals_config;
else
    FileName_visuals_config = Parser.Results.FileName_visuals_config;
end
    
visuals_config = load(FileName_visuals_config);
visuals_config = visuals_config.visuals_config;

switch Parser.Results.DrawRobot_Type
    case 'Default'
        DrawRobot_abstract = @(SimulationEngine, old_h) SRD_DrawRobot_Default(SimulationEngine, old_h, ...
            visuals_config.DrawRobot_Default_RobotColor, ...
            visuals_config.DrawRobot_Default_EdgeAlpha, ...
            visuals_config.DrawRobot_Default_FaceAlpha, ...
            visuals_config.DrawRobot_Default_LineWidth);
    case 'STL'
        DrawRobot_abstract = @(SimulationEngine, old_h) SRD_DrawRobot_STL(SimulationEngine, old_h, ...
            visuals_config.DrawRobot_STL_FaceColor, ...
            visuals_config.DrawRobot_STL_EdgeColor, ...
            visuals_config.DrawRobot_STL_FaceLighting, ...
            visuals_config.DrawRobot_STL_AmbientStrength);
    case 'Custom'
        DrawRobot_abstract = @(SimulationEngine, old_h) Parser.Results.DrawRobot_Custom_handle(SimulationEngine, old_h);
    otherwise
        error('Undefined drawing type');
end

ToDrawFrames = visuals_config.ToDrawFrames;

if ToDrawFrames
    DrawRobot_Frames = @(old_h) SRD_DrawRobot_Frames(SimulationEngine, old_h, ...
        visuals_config.DrawRobot_Frame_Scale, ...
        visuals_config.DrawRobot_Frame_LineWidth);
else
    DrawRobot_Frames = [];
end


    function h = DrawGivenPosition(q, old_h, SimulationEngine, DrawRobot_abstract, ToDrawFrames, DrawRobot_Frames)
        SimulationEngine.Update(q);
        h = DrawRobot_abstract(SimulationEngine, old_h);
        if ToDrawFrames
            h = DrawRobot_Frames(h);
        end
    end

    function h = DrawInitialPosition(old_h, SimulationEngine, DrawRobot_abstract, ToDrawFrames, DrawRobot_Frames)
        SimulationEngine.Update(SimulationEngine.IC.q);
        h = DrawRobot_abstract(SimulationEngine, old_h);
        if ToDrawFrames
            h = DrawRobot_Frames(h);
        end
    end

    function h = DrawCurrentPosition(old_h, SimulationEngine, DrawRobot_abstract, ToDrawFrames, DrawRobot_Frames)
        h = DrawRobot_abstract(SimulationEngine, old_h);
        if ToDrawFrames
            h = DrawRobot_Frames(h);
        end
    end


switch Parser.Results.Function_Type
    case 'DrawGivenPosition'
        DrawRobot_function = @(q, old_h)DrawGivenPosition(q, old_h, SimulationEngine, DrawRobot_abstract, ToDrawFrames, DrawRobot_Frames);
    case 'DrawInitialPosition'
        DrawRobot_function = @(old_h)DrawInitialPosition(old_h,  SimulationEngine, DrawRobot_abstract, ToDrawFrames, DrawRobot_Frames);
    case 'DrawCurrentPosition'
        DrawRobot_function = @(old_h)DrawCurrentPosition(old_h,  SimulationEngine, DrawRobot_abstract, ToDrawFrames, DrawRobot_Frames);
end

end