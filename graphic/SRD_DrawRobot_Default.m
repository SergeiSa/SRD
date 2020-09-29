function h = SRD_DrawRobot_Default(SimulationEngine, old_h, ...
    RobotColor, EdgeAlpha, FaceAlpha, LineWidth)

n = size(SimulationEngine.LinkArray, 1);
index = 0;

if isempty(old_h)
    h.Default = cell(n, 1);
else
    if isfield(old_h, Default)
        h.Default = old_h.Default;
    else
        h.Default = cell(n, 1);
    end
end

for i = 1:n
    if SimulationEngine.LinkArray(i).Order > 0
        index = index + 1;
        
        Polygon = [SimulationEngine.LinkArray(i).AbsoluteBase, SimulationEngine.LinkArray(i).AbsoluteFollower]; 
        
        if isempty(h.Default{i})
            h.Default{i} = fill3(Polygon(1, :)', Polygon(2, :)', Polygon(3, :)', RobotColor, 'EdgeAlpha', EdgeAlpha, 'FaceAlpha', FaceAlpha, ...
                'LineWidth', LineWidth); hold on;
        else
            h.Default{i}.Vertices = Polygon';
        end
    end
end
end