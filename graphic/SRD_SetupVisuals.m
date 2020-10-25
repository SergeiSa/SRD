function SRD_SetupVisuals(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_SetupVisuals';
Parser.addOptional('AxisLimits', [-1; 1; -1; 1; -1; 1]);
Parser.addOptional('ViewAngle', [-37.5, 30]);
Parser.addOptional('ToDrawFrames', false);
Parser.addOptional('ToDrawMeshes', false);

Parser.addOptional('Animation_ToUseGrid', true);
Parser.addOptional('Animation_ToUseGridMinor', true);

Parser.addOptional('DrawRobot_Default_RobotColor', [0.6 0.3 0]);
Parser.addOptional('DrawRobot_Default_EdgeAlpha', 0.3);
Parser.addOptional('DrawRobot_Default_FaceAlpha', 1);
Parser.addOptional('DrawRobot_Default_LineWidth', 0.5);

Parser.addOptional('DrawRobot_STL_FaceColor', [0.8 0.8 1.0]);
Parser.addOptional('DrawRobot_STL_EdgeColor', 'none');
Parser.addOptional('DrawRobot_STL_FaceLighting', 'gouraud');
Parser.addOptional('DrawRobot_STL_AmbientStrength', 0.15);
Parser.addOptional('DrawRobot_STL_camlight', 'headlight');
Parser.addOptional('DrawRobot_STL_material', 'dull');

Parser.addOptional('DrawRobot_Frame_Scale', 0.2);
Parser.addOptional('DrawRobot_Frame_LineWidth', 1);

Parser.addOptional('FileName_visuals_config', 'datafile_visuals_config.mat');
Parser.parse(varargin{:});

visuals_config.AxisLimits = Parser.Results.AxisLimits;
visuals_config.ViewAngle = Parser.Results.ViewAngle;
visuals_config.AxisLimits = Parser.Results.AxisLimits;

visuals_config.ToDrawFrames = Parser.Results.ToDrawFrames;
visuals_config.ToDrawMeshes = Parser.Results.ToDrawMeshes;

visuals_config.Animation_ToUseGrid = Parser.Results.Animation_ToUseGrid;
visuals_config.Animation_ToUseGridMinor = Parser.Results.Animation_ToUseGridMinor;

visuals_config.DrawRobot_Default_RobotColor = Parser.Results.DrawRobot_Default_RobotColor;
visuals_config.DrawRobot_Default_EdgeAlpha = Parser.Results.DrawRobot_Default_EdgeAlpha;
visuals_config.DrawRobot_Default_FaceAlpha = Parser.Results.DrawRobot_Default_FaceAlpha;
visuals_config.DrawRobot_Default_LineWidth = Parser.Results.DrawRobot_Default_LineWidth;

visuals_config.DrawRobot_STL_FaceColor = Parser.Results.DrawRobot_STL_FaceColor;
visuals_config.DrawRobot_STL_EdgeColor = Parser.Results.DrawRobot_STL_EdgeColor;
visuals_config.DrawRobot_STL_FaceLighting = Parser.Results.DrawRobot_STL_FaceLighting;
visuals_config.DrawRobot_STL_AmbientStrength = Parser.Results.DrawRobot_STL_AmbientStrength;
visuals_config.DrawRobot_STL_camlight = Parser.Results.DrawRobot_STL_camlight;
visuals_config.DrawRobot_STL_material = Parser.Results.DrawRobot_STL_material;

visuals_config.DrawRobot_Frame_Scale = Parser.Results.DrawRobot_Frame_Scale;
visuals_config.DrawRobot_Frame_LineWidth = Parser.Results.DrawRobot_Frame_LineWidth;

save(Parser.Results.FileName_visuals_config, 'visuals_config');
end