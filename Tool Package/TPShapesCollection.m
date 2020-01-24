classdef TPShapesCollection < handle
    properties
    end
    methods
        function Output = GetCylinder(obj, varargin)
            %input options (name-value pairs)
            p = inputParser;
            p.FunctionName = 'TPShapesCollection.GetCylinder';
            p.addOptional('Center', [0; 0; 0], @(x) size(x, 1)==3);
            p.addOptional('Radius', 1, @isscalar);
            p.addOptional('Length', 1, @isscalar);
            p.addOptional('RotationMatrix', eye(3));
            
            p.addOptional('StepSizeLength', 0.1, @isscalar);
            p.addOptional('StepSizePhi', 0.2, @isscalar);
            
            p = obj.AddParameters(p);
            p.parse(varargin{:});
            
            CountL = floor(p.Results.Length / p.Results.StepSizeLength);
            CountPhi = floor(2*pi / p.Results.StepSizePhi) + 2;
            
            Math = MathClass;
            
            %construct 2-dimentional arrays X Y Z
            if p.Results.ReturnXYZarrays
                Output.X = zeros(CountL, CountPhi);
                Output.Y = zeros(CountL, CountPhi);
                Output.Z = zeros(CountL, CountPhi);
                
                for i = 1:CountL
                    for j = 1:CountPhi
                        
                        phi = j*p.Results.StepSizePhi;
                        r = Math.RotationMatrix2D(phi) * [p.Results.Radius; 0];
                        L = i*p.Results.StepSizeLength;
                        
                        Point = p.Results.Center + p.Results.RotationMatrix * [r; L];
                        
                        Output.X(i, j) = Point(1);
                        Output.Y(i, j) = Point(2);
                        Output.Z(i, j) = Point(3);
                    end
                end
                
                %draw surface
                if p.Results.Draw
                    obj.DrawSurface(p, Output);
                end
            end
            
            %construct Nx3 array of points
            if p.Results.ReturnPointCloud
                Output.PointCloud = zeros(CountL*CountPhi, 3);
                
                for i = 1:CountL
                    for j = 1:CountPhi
                        
                        phi = j*p.Results.StepSizePhi;
                        r = Math.RotationMatrix2D(phi) * [p.Results.Radius; 0];
                        L = i*p.Results.StepSizeLength;
                        
                        Point = p.Results.Center + p.Results.RotationMatrix * [r; L];
                        
                        index = (i - 1)*CountPhi + j;
                        Output.PointCloud(index, :) = Point;
                    end
                end
                
                %draw point cloud
                if p.Results.Draw
                    obj.DrawPointCloud(p, Output);
                end
            end 
        end
        
        function Output = GetPlaneSegment(obj, varargin)
            %input options (name-value pairs)
            p = inputParser;
            p.FunctionName = 'TPShapesCollection.GetPlaneSegment';
            p.addOptional('LengthX', 1, @isscalar);
            p.addOptional('LengthY', 1, @isscalar);
            p.addOptional('RotationMatrix', eye(3));
            p.addOptional('Shift', [0; 0; 0]);
            
            p.addOptional('StepSizeX', 0.1, @isscalar);
            p.addOptional('StepSizeY', 0.1, @isscalar);
            
            p = obj.AddParameters(p);
            p.parse(varargin{:});
            
            dx = p.Results.StepSizeX;
            dy = p.Results.StepSizeY;
            
            CountX = floor(p.Results.LengthX / dx);
            CountY = floor(p.Results.LengthY / dy);
            
            %construct 2-dimentional arrays X Y Z
            if p.Results.ReturnXYZarrays
                Output.X = zeros(CountX, CountY);
                Output.Y = zeros(CountX, CountY);
                Output.Z = zeros(CountX, CountY);
                
                for i = 1:CountX
                    for j = 1:CountY
                        
                        R = [i*dx; j*dy; 0] + p.Results.Shift;
                        Point = p.Results.RotationMatrix * R;
                        
                        Output.X(i, j) = Point(1);
                        Output.Y(i, j) = Point(2);
                        Output.Z(i, j) = Point(3);
                    end
                end
                
                %draw surface
                if p.Results.Draw
                    obj.DrawSurface(p, Output);
                end
            end
            
            %construct Nx3 array of points
            if p.Results.ReturnPointCloud
                Output.PointCloud = zeros(CountX*CountY, 3);
                
                for i = 1:CountX
                    for j = 1:CountY
                        
                        R = [i*dx; j*dy; 0] + p.Results.Shift;
                        Point = p.Results.RotationMatrix * R;
                        
                        index = (i - 1)*CountY + j;
                        Output.PointCloud(index, :) = Point;
                    end
                end
                
                %draw point cloud
                if p.Results.Draw
                    obj.DrawPointCloud(p, Output);
                end
            end 
        end        
 

        function Output = GetTriangularPrism(obj, varargin)
            %input options (name-value pairs)
            p = inputParser;
            p.FunctionName = 'TPShapesCollection.GetTriangularPrism';
            p.addOptional('Triangle', [-1, 1, 0; -1, -1, 1; 0, 0, 0]);
            p.addOptional('Height', 1, @isscalar);
            p.addOptional('RotationMatrix', eye(3));
            p.addOptional('Shift', [0; 0; 0]);
            
            p.addOptional('ReturnVertices', true);
            
            p = obj.AddParameters(p);
            p.parse(varargin{:});
            
            A = p.Results.Triangle(:, 1);
            B = p.Results.Triangle(:, 2);
            C = p.Results.Triangle(:, 3);
            
            h = cross((B - A), (C- A));
            h = p.Results.Height * h / norm(h);
            
            V = [(A-h/2), (B-h/2), (C-h/2), (A+h/2), (B+h/2), (C+h/2)];
            for i = 1:6
                V(:, i) =  p.Results.RotationMatrix * V(:, i) + p.Results.Shift;
            end
            
            Output.Vertices = V';
            Output.Faces = [1, 2, 3, 3; 
                            4, 5, 6, 6; 
                            1, 2, 5, 4; 
                            2, 3, 6, 5; 
                            3, 1, 4, 6];
            Output.RotationMatrix = p.Results.RotationMatrix;
            Output.Shift = p.Results.Shift;
            
            %construct 2-dimentional arrays X Y Z
            if p.Results.ReturnXYZarrays
                warning('XYZ arrays generation not implemented for this shape');
            end
            
            %construct Nx3 array of points
            if p.Results.ReturnPointCloud
                warning('Point cloud arrays generation not implemented for this shape');
            end 
            
            %draw surface
            if p.Results.Draw
                %obj.DrawSurface(p, Output);
                patch('Faces', Output.Faces, 'Vertices' , Output.Vertices, 'FaceColor', p.Results.FaceColor, ...
                    'FaceAlpha', p.Results.FaceAlpha, 'EdgeColor', p.Results.EdgeColor);
                hold on;
                plot3(Output.Vertices(:, 1), Output.Vertices(:, 2), Output.Vertices(:, 3), p.Results.Marker, ...
                'MarkerEdgeColor', p.Results.MarkerEdgeColor, 'MarkerFaceColor', p.Results.MarkerFaceColor);
            end
        end             
        
        
        function Output = GetCircle(obj, varargin)
            %input options (name-value pairs)
            p = inputParser;
            p.FunctionName = 'TPShapesCollection.GetCircle';
            p.addOptional('Center', [0; 0; 0], @(x) size(x, 1)==3);
            p.addOptional('Radius', 1, @isscalar);
            p.addOptional('RotationMatrix', eye(3));
            
            p.addOptional('StepSizePhi', 0.2, @isscalar);
            
            p = obj.AddParameters(p);
            p.parse(varargin{:});
            
            CountPhi = floor(2*pi / p.Results.StepSizePhi) + 2;
            
            Math = MathClass;
            
            %construct 2-dimentional arrays X Y Z
            if p.Results.ReturnXYZarrays
                Output.X = zeros(CountPhi, 1);
                Output.Y = zeros(CountPhi, 1);
                Output.Z = zeros(CountPhi, 1);
                
                for j = 1:CountPhi
                    
                    phi = j*p.Results.StepSizePhi;
                    r = Math.RotationMatrix2D(phi) * [p.Results.Radius; 0];
                    
                    Point = p.Results.Center + p.Results.RotationMatrix * [r; 0];
                    
                    Output.X(j, 1) = Point(1);
                    Output.Y(j, 1) = Point(2);
                    Output.Z(j, 1) = Point(3);
                end
                
                %draw surface
                if p.Results.Draw
                    fill3(Output.X, Output.Y, Output.Z,'r', 'FaceColor', p.Results.FaceColor, 'FaceAlpha', p.Results.FaceAlpha, ...
                        'EdgeAlpha', p.Results.EdgeAlpha);
                    hold on;
                end
            end
            
            %construct Nx3 array of points
            if p.Results.ReturnPointCloud
                Output.PointCloud = zeros(CountPhi, 3);
                
                for j = 1:CountPhi
                    phi = j*p.Results.StepSizePhi;
                    r = Math.RotationMatrix2D(phi) * [p.Results.Radius; 0];
                    
                    Point = p.Results.Center + p.Results.RotationMatrix * [r; 0];
                    
                    Output.PointCloud(j, :) = Point;
                end
                
                %draw point cloud
                if p.Results.Draw
                    obj.DrawPointCloud(p, Output);
                end
            end 
        end              
        
        
        
        %adds standard input parameters
        function p = AddParameters(~, p)
            p.addOptional('ReturnXYZarrays', true);
            p.addOptional('ReturnPointCloud', false);
            
            p.addOptional('Draw', true);
            p.addOptional('MarkerEdgeColor', 'g');
            p.addOptional('MarkerFaceColor', 'b');
            p.addOptional('Marker', 'o');
            p.addOptional('FaceColor', 'r');
            p.addOptional('EdgeColor', 'k');
            p.addOptional('FaceAlpha', 1, @isscalar);
            p.addOptional('EdgeAlpha', 1, @isscalar);
            p.addOptional('Figure', []);
        end
        
        %draws surface using generated data
        function DrawSurface(~, p, Output)
            if ~isempty(p.Results.Figure)
                figure(p.Results.Figure);
            end
            surf(Output.X, Output.Y, Output.Z, 'FaceColor', p.Results.FaceColor, 'FaceAlpha', p.Results.FaceAlpha, ...
                'EdgeAlpha', p.Results.EdgeAlpha);
            hold on;
        end
        
        %draws point cloud using generated data
        function DrawPointCloud(~, p, Output)
            if ~isempty(p.Results.Figure)
                figure(p.Results.Figure);
            end
            plot3(Output.PointCloud(:, 1), Output.PointCloud(:, 2), Output.PointCloud(:, 3), p.Results.Marker, ...
                'MarkerEdgeColor', p.Results.MarkerEdgeColor, 'MarkerFaceColor', p.Results.MarkerFaceColor);
            hold on;
        end
        
    end
end
    