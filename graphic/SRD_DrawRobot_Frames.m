function h = SRD_DrawRobot_Frames(SimulationEngine, old_h, FrameScale, LineWidth)
n = size(SimulationEngine.LinkArray, 1);

if isempty(old_h)
    h.Frames = cell(n, 1);
else
    if isfield(old_h, 'Frames')
        h.Frames = old_h.Frames;
    else
        h.Frames = cell(n, 3);
    end
end

for i = 1:n
    E = eye(3)*FrameScale;
    translation = SimulationEngine.LinkArray(i).AbsoluteBase;
    E = SimulationEngine.LinkArray(i).AbsoluteOrientation * E + translation;
    E = E';
    if isempty(h.Frames{i})
        h.Frames{i, 1} = plot3([translation(1),E(1,1)], [translation(2),E(1,2)], [translation(3),E(1,3)], ...
            'Color', 'r', 'LineWidth', LineWidth);
        h.Frames{i, 2} = plot3([translation(1),E(2,1)], [translation(2),E(2,2)], [translation(3),E(2,3)], ...
            'Color', 'g', 'LineWidth', LineWidth);
        h.Frames{i, 3} = plot3([translation(1),E(3,1)], [translation(2),E(3,2)], [translation(3),E(3,3)], ...
            'Color', 'b', 'LineWidth', LineWidth);
    else
        error('implement_me')
    end
end
end