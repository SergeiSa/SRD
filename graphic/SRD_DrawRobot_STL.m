function h = SRD_DrawRobot_STL(SimulationEngine, old_h, ...
    FaceColor, EdgeColor, FaceLighting, AmbientStrength)

n = size(SimulationEngine.LinkArray, 1);
index = 0;

if isempty(old_h)
    h.STL = cell(n, 1);
else
    if isfield(old_h, STL)
        h.STL = old_h.STL;
    else
        h.STL = cell(n, 1);
    end
end

%             camlight('headlight');
%             material('dull');

for i = 1:n
    if obj.SimulationEngine.LinkArray(i).Order > 0
        index = index + 1;
        if isempty(SimulationEngine.LinkArray(i).StlPath)
            continue;
        end
        
        Vertices = SimulationEngine.LinkArray(i).Mesh.Vertices;
        
        Polygon = struct();
        Polygon.vertices = SimulationEngine.LinkArray(i).AbsoluteOrientation * Vertices'...
            + SimulationEngine.LinkArray(i).AbsoluteBase;
        Polygon.vertices = Polygon.vertices';
        
        if isempty(old_h)
            Polygon.faces = SimulationEngine.LinkArray(i).Mesh.Faces;
            
            h.STL{i} = patch(Polygon, ...
                'FaceColor',       FaceColor,        ...
                'EdgeColor',       EdgeColor,        ...
                'FaceLighting',    FaceLighting,     ...
                'AmbientStrength', AmbientStrength); hold on;
        else
            h.STL{i}.Vertices = Polygon.vertices;
        end
        
    end
end

end