function SRD__make_default_scene(Type, FileName_visuals_config)

if nargin < 2
    FileName_visuals_config = 'datafile_visuals_config.mat';
end
visuals_config = load(FileName_visuals_config);
visuals_config = visuals_config.visuals_config;


if visuals_config.Animation_ToUseGrid
    grid on;
end
if visuals_config.Animation_ToUseGridMinor
    grid minor;
end
if visuals_config.Animation_ToUseGridMinor
    grid minor;
end

switch Type
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

view(visuals_config.ViewAngle);
axis(visuals_config.AxisLimits);

axis equal;
end