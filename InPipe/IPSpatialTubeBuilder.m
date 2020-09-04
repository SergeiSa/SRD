classdef IPSpatialTubeBuilder < handle
    properties
        %function handle to the center line function
        CenterLine = [];
        
        %function handle to the diameter line function
        Radius = [];
        
        %%%%%%%%%%%%%%%
        %%% Limits and discretisation
        
        %The parameter values defining the beginning and the end of teh
        %centerline
        CenterLineStart;
        CenterLineEnd;
        
        %center line discretization step
        CenterLineDiscretizationStep = 0.05;
        RadialDiscretizationStep = 0.2;
        
        %%%%%%%%%%%%%%%
        %%% Data
        
        %data array that represents the center line
        CenterLineData = [];
        
        %data array that represents the curved tube
        TubeData = [];
        
        %data array that represents the flattened tube
        FlatTubeData = [];
        
        
        %%%%%%%%%%%%%%%
        %%% Obstacles
        
        Obstacles;
        
        %%%%%%%%%%%%%%%
        %%% graphics
        
        %the figure used for animation
        drawing_figure = [];
        
        TubeFaceColor = [0.3, 0.5, 0.3];
        
        TubeFaceAlpha = 1;
        
        %%%%%%%%%%%%%%%
        %%% others
        
        %MathClass object
        Math;
        
    end
    methods
        function obj = IPSpatialTubeBuilder(CenterLine, Radius, Range)
            
            if nargin < 3
                obj.CenterLineStart = 0;
                obj.CenterLineEnd = 1;
            else
                obj.CenterLineStart = Range(1);
                obj.CenterLineEnd = Range(2);
            end
            
            obj.CenterLine = CenterLine;
            obj.Radius = Radius;
            
            obj.Math = MathClass;
        end
        
        %generates the center line data
        function GenerateCenterLine(obj)
            Length = obj.CenterLineEnd - obj.CenterLineStart;
            Count = floor(Length / obj.CenterLineDiscretizationStep);
            obj.CenterLineData.Coordinates = zeros(Count, 3);
            obj.CenterLineData.Parameter = zeros(Count, 1);
            
            obj.CenterLineData.Radius = zeros(Count, 1);
            obj.CenterLineData.Direction = zeros(Count, 3);
            obj.CenterLineData.TangentBasis = zeros(Count, 6);
            
            for i = 1:Count
                s = obj.CenterLineStart + i*obj.CenterLineDiscretizationStep;
                r1 = obj.CenterLine(s);
                d = obj.Radius(s);
                
                %if this condition fails we don't update Direction, which
                %should only happen at the end of the center line
                if (s + obj.CenterLineDiscretizationStep) < obj.CenterLineEnd
                    r2 = obj.CenterLine(s + obj.CenterLineDiscretizationStep);
                    NewDirection = (r2 - r1) / norm(r2 - r1);
                else
                    NewDirection = OldDirection;
                end
                
                if i == 1
                    TangentBasis = null(NewDirection');
                else
                    Axis = cross(OldDirection, NewDirection);
                    if norm(Axis) ~= 0
                        Axis = Axis / norm(Axis);
                        theta = acos(dot(OldDirection, NewDirection));
                        
                        T = obj.Math.RotationAroundAxis(Axis, theta);
                        TangentBasis = T*TangentBasis;
                    end
                end
                
                OldDirection = NewDirection; %save for the next iteration
                
                obj.CenterLineData.Parameter(i) = s;
                obj.CenterLineData.Coordinates(i, :) = r1';
                
                obj.CenterLineData.Radius(i) = d;
                obj.CenterLineData.Direction(i, :) = NewDirection';
                obj.CenterLineData.TangentBasis(i, 1:3) = TangentBasis(:, 1)';
                obj.CenterLineData.TangentBasis(i, 4:6) = TangentBasis(:, 2)';
            end
        end
        
        %this function generates the data used to draw the tube
        function GenerateTube(obj)
            if isempty(obj.CenterLineData)
                obj.GenerateCenterLine;
            end
            
            Count = size(obj.CenterLineData.Parameter, 1);
            CountPhi = floor(2*pi / obj.RadialDiscretizationStep) + 2;
            
            X_3D = zeros(Count, CountPhi);
            Y_3D = zeros(Count, CountPhi);
            Z_3D = zeros(Count, CountPhi);
            
            X_2D = zeros(Count, CountPhi);
            Y_2D = zeros(Count, CountPhi);
            Z_2D = zeros(Count, CountPhi);
            
            NumberOfObstacles = length(obj.Obstacles);
            
            for i = 1:Count
                s = obj.CenterLineData.Parameter(i);
                CenterLinePoint = obj.CenterLineData.Coordinates(i, :)';
                R = obj.CenterLineData.Radius(i);
                Direction = obj.CenterLineData.Direction(i, :)';
                BasisVector = obj.CenterLineData.TangentBasis(i, 1:3)';
                for j = 1:CountPhi
                    phi = j*obj.RadialDiscretizationStep;
                    T = obj.Math.RotationAroundAxis(Direction, phi);
                    
                    Vector = T*BasisVector;
                    Point = CenterLinePoint + R*Vector;
                    
                    
                    InObstacle = false;
                    for k = 1:NumberOfObstacles
                        
                        ObstaclePosition = obj.Obstacles{k}.Position;
                        
                        %below we try to deal with the problem of angle 0
                        %being 'far' from angle 2*pi in Euclidean metric
                        phi1 = ObstaclePosition(2);
                        if phi1 < pi/2
                            phi_shift = pi/2;
                        end
                        if phi1 > 3*pi/2
                            phi_shift = -pi/2;
                        end
                        phi1 = obj.Math.AnglesIn_0to2pi(phi1+phi_shift);
                        phi2 = obj.Math.AnglesIn_0to2pi(phi+phi_shift);
                        
                        delta = [ObstaclePosition(1); phi1] - [s; phi2];
                        if (delta' * obj.Obstacles{k}.Weight * delta) < obj.Obstacles{k}.Radius
                            InObstacle = true;
                        end
                    end
                    if InObstacle
                        X_3D(i, j) = NaN;
                        Y_3D(i, j) = NaN;
                        Z_3D(i, j) = NaN;
                        
                        X_2D(i, j) = NaN;
                        Y_2D(i, j) = NaN;
                        Z_2D(i, j) = NaN;
                    else
                        X_3D(i, j) = Point(1);
                        Y_3D(i, j) = Point(2);
                        Z_3D(i, j) = Point(3);
                        
                        X_2D(i, j) = s;
                        Y_2D(i, j) = phi;
                        Z_2D(i, j) = -R;
                    end
                    
                end
            end
            
            obj.TubeData.X_3D = X_3D;
            obj.TubeData.Y_3D = Y_3D;
            obj.TubeData.Z_3D = Z_3D;
            
            obj.TubeData.X_2D = X_2D;
            obj.TubeData.Y_2D = Y_2D;
            obj.TubeData.Z_2D = Z_2D;
        end
        
        %adds holes to the tube
        function BoreHoles(obj, Centers, Radii, DeformationMatrix)
            
            for i = 1:size(obj.TubeData.X_3D, 1)
                for j = 1:size(obj.TubeData.X_3D, 2)
                    
                    InHole = false;
                    for k = 1:size(Centers, 1)
                        delta = [i-Centers(k, 1); j-Centers(k, 2)];
                        distance = delta' * DeformationMatrix * delta;
                        if distance < Radii(k)
                            InHole = true;
                        end
                    end
                    if InHole
                        obj.TubeData.X_3D(i, j) = NaN;
                        obj.TubeData.Y_3D(i, j) = NaN;
                        obj.TubeData.Z_3D(i, j) = NaN;
                    end
                end
            end
        end
        
        %generates a point cloud
        function GeneratePointCloud(obj)
            n1 = size(obj.TubeData.X_3D, 1);
            n2 = size(obj.TubeData.X_3D, 2);
            PointCloud = zeros(n1, n2, 3);
            PointCloud(:, :, 1) = obj.TubeData.X_3D;
            PointCloud(:, :, 2) = obj.TubeData.Y_3D;
            PointCloud(:, :, 3) = obj.TubeData.Z_3D;
            obj.TubeData.PointCloud = reshape(PointCloud, [n1*n2, 3]);
        end
        
        %height map for the unwrapped representation of the tube
        function h = HeightMap(obj, input1, ~)
            if nargin < 3
                s = input1(1);
            else
                s = input1;
            end
            h = -obj.Radius(s);
        end
        
        %maps from the unwrapped tube to the initial tube
        function MappedPoint = Map_onto_3D_tube(obj, Point)
            s = Point(1);
            phi = Point(2);
            radius = Point(3);
            
            CenterLinePoint = obj.CenterLine(s);
            
            index = obj.Math.FindPlaceInArray(obj.CenterLineData.Parameter, s, 'Closest');
            BasisVector = obj.CenterLineData.TangentBasis(index, 1:3)';
            Direction = obj.CenterLineData.Direction(index, :)';
            
            T = obj.Math.RotationAroundAxis(Direction, phi);
            
            MappedPoint = CenterLinePoint - radius*T*BasisVector;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%% graphics 
        
        %sets the figure for drawing
        function SetFigure(obj)
           if isempty(obj.drawing_figure)
               obj.drawing_figure = figure();
           else
               figure(obj.drawing_figure);
           end
           
           obj.drawing_figure.Color = 'w';
        end
        
        %draws the center line of the tube
        function h = DrawCenterLine(obj)
            if isempty(obj.CenterLineData)
                obj.GenerateCenterLine;
            end
            
            obj.SetFigure();
            
            h = plot3(obj.CenterLineData.Coordinates(:, 1), obj.CenterLineData.Coordinates(:, 2), ...
                obj.CenterLineData.Coordinates(:, 3), 'LineWidth', 3);
            grid on;
            axis equal;
            hold on;
        end
        
        %animates the center line
        function AnimateCenterLine(obj)
            h = DrawCenterLine(obj);
            
            Count = size(obj.CenterLineData.Parameter, 1);
            for i = 1:Count
                CenterLinePoint = obj.CenterLineData.Coordinates(i, :)';
                Direction = 0.1*obj.CenterLineData.Direction(i, :)';
                TB1 = 0.1*obj.CenterLineData.TangentBasis(i, 1:3)';
                TB2 = 0.1*obj.CenterLineData.TangentBasis(i, 4:6)';
                DirectionVector = [CenterLinePoint'; (CenterLinePoint + Direction)'];
                TB1Vector = [CenterLinePoint'; (CenterLinePoint + TB1)'];
                TB2Vector = [CenterLinePoint'; (CenterLinePoint + TB2)'];
                
                h1 = plot3(CenterLinePoint(1), CenterLinePoint(2), CenterLinePoint(3), 'o', 'MarkerEdgeColor', 'k', ...
                    'MarkerFaceColor', 'r');
                h2 = plot3(DirectionVector(:, 1), DirectionVector(:, 2), DirectionVector(:, 3), 'LineWidth', 2, ...
                    'Color', 'g');
                h3 = plot3(TB1Vector(:, 1), TB1Vector(:, 2), TB1Vector(:, 3), 'LineWidth', 2, ...
                    'Color', [1 0 0]);
                h4 = plot3(TB2Vector(:, 1), TB2Vector(:, 2), TB2Vector(:, 3), 'LineWidth', 2, ...
                    'Color', [1 0.5 0]);
                
                pause(0.01);
                delete(h1); delete(h2); delete(h3); delete(h4);
            end
        end
        
        %draws the tube
        function h = DrawTube3D(obj)
            
            if isempty(obj.TubeData)
                obj.GenerateTube;
            end
            
            obj.SetFigure();
            
            surf(obj.TubeData.X_3D, obj.TubeData.Y_3D, obj.TubeData.Z_3D, 'FaceLighting', 'gouraud', ...
                'EdgeAlpha', 0.3, 'FaceColor', obj.TubeFaceColor, 'FaceAlpha', obj.TubeFaceAlpha);
            hold on;
            light('Position',[0.5 -0.5 0]*2,'Style','local');
            light('Position',[-2 -2 2]*2,'Style','local');
            light('Position',[-2 3 4]*2,'Style','local');
            light('Position',[1 -2 0.5]*2,'Style','local');
            grid on;
            axis equal;
        end
        
        %draws the tube's map
        function h = DrawTube2D(obj)
            
            if isempty(obj.TubeData)
                obj.GenerateTube;
            end
            
            obj.SetFigure();
            
            surf(obj.TubeData.X_2D, obj.TubeData.Y_2D, obj.TubeData.Z_2D, 'FaceLighting', 'gouraud', ...
                'EdgeAlpha', 0.3);
            hold on;
            light('Position',[0.5 -0.5 1],'Style','local');
            light('Position',[-2 -2 2],'Style','local');
            light('Position',[-2 3 4],'Style','local');
            grid on;
            axis equal;
        end      
        
        
        
        
        
        
        
    end
end
    