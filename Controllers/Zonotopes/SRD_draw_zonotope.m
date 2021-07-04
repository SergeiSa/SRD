function handle = SRD_draw_zonotope(G, h, varargin)
Parser = inputParser;
Parser.FunctionName = 'draw_zonotope';
Parser.addOptional('FaceColor', [0.8, 0.2, 0.2]);
Parser.addOptional('FaceAlpha', 0.3);
Parser.parse(varargin{:});

        Vertices = SRD_zonotope_get_points_vertices(G) + h;
        
        try
        indices_convhull = convhull(Vertices');
        Vertices = Vertices(:, indices_convhull);
        
        handle = fill(Vertices(1, :)', Vertices(2, :)', ...
            Parser.Results.FaceColor, ...
            'FaceAlpha', Parser.Results.FaceAlpha); hold on;
        catch
            warning('something is up with teh zonotope, maybe it is empty')
        end
end