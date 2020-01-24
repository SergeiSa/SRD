classdef SPStereographicProjectionPlaneTiling < handle
    properties
        
        %%%%%%%%%%%%%%%%
        %%% objects of other classes
        
        StereographicProjectionFreeRegionsObj;
        %object of StereographicProjectionFreeRegions class
        
        Math;
        %object of MathClass class
        
        %%%%%%%%%%%%%%%%
        %%% data array
        
        HeightMapArrays = [];
        % a structure with fields .X .Y .Z compatible with surf function
        
        ObstaclePointArray = [];
        %an array with two columns representing x and y coordinates of
        %obstacles
        
        %%%%%%%%%%%%%%%%
        %%% algorithm settings
        
        height_variation_tolerance = 0.01;
        % defines the threshold height difference between a point on a
        % height map and the CenterPoint after which the point will be
        % considered an obstacle;
        
        default_boundary_density = 100;
        
        %%%%%%%%%%%%%%%%
        %%% graphics
        
        drawing_figure = [];
        
        drawing_obstacles_height = 0;
        %deternines z coordinate of the obstacles from ObstaclePointArray
        %when they are drawn
        
        LightPoisitionArray = [0 0 2; 1 1 2];
        %defines positions of the lights and their number
        
    end
    methods
        function obj = SPStereographicProjectionPlaneTiling()
            obj.StereographicProjectionFreeRegionsObj = SPStereographicProjectionFreeRegions;
            obj.Math = MathClass;
        end
        
        %adds new obstacle points
        function AddObstaclePoints(obj, Obstacles)
            obj.ObstaclePointArray = [obj.ObstaclePointArray; Obstacles];
        end
        
        %creates an array of obstacles from the .HeightMapArrays;
        %oll points that have more than .height_variation_tolerance height
        %difference with the desired_height are considered obstacles
        function Obstacles = HeightMap2Obstacles(obj, desired_height)
            [N, M] = size(obj.HeightMapArrays.Z);
            Z = obj.HeightMapArrays.Z - desired_height*ones(N, M);
            
            Obstacles = zeros(N*M, 2); index = 0;
            for i = 1:N
                for j = 1:M
                    h = Z(i, j);
                    if isnan(h) || (abs(h) > obj.height_variation_tolerance)
                        index = index + 1;
                        Obstacles(index, :) = [obj.HeightMapArrays.X(i, j), obj.HeightMapArrays.Y(i, j)];
                    end
                end
            end
            Obstacles = Obstacles(1:index, :);
            
            if nargout < 1
                obj.AddObstaclePoints(Obstacles);
            end
        end
        
        %adds boundy made of obstacles to the .ObstaclePointArray;
        %
        %NumberOfPoints - the total number of points in the boundary,
        %if not specified the .default_boundary_density property will
        %be used to calculate it.
        %
        %if no output specified the function will add the boundary to
        %ObstaclePointArray, if there is an output - it won't do that
        function Obstacles = AddBoundary(obj, NumberOfPoints)
            [N, M] = size(obj.HeightMapArrays.X);
            
            P{1} = [obj.HeightMapArrays.X(1, 1); obj.HeightMapArrays.Y(1, 1)];
            P{2} = [obj.HeightMapArrays.X(1, M); obj.HeightMapArrays.Y(1, M)];
            P{3} = [obj.HeightMapArrays.X(N, M); obj.HeightMapArrays.Y(N, M)];
            P{4} = [obj.HeightMapArrays.X(N, 1); obj.HeightMapArrays.Y(N, 1)];            
            
            L(1, 1) = norm(P{1} - P{2});
            L(2, 1) = norm(P{2} - P{3});
            L(3, 1) = norm(P{3} - P{4});
            L(4, 1) = norm(P{4} - P{1});
            Length = sum(L);
            
            if nargin < 2
                NumberOfPoints = Length*obj.default_boundary_density;
            end
            
            Count = floor(NumberOfPoints * L / Length);
            Count_total = sum(Count);
            Obstacles = zeros(Count_total, 2);
            
            index = 1;
            for k = 1:4
                k2 = k + 1;
                if k2 > 4
                    k2 = 1;
                end
                for i = 1:Count(k)
                    Obstacles(index, :) = P{k} * (Count(k) - i) / Count(k) + P{k2} * i / Count(k);
                    index = index + 1;
                end
            end
            
            if nargout < 1
                obj.AddObstaclePoints(Obstacles);
            end
        end
        
        %generates an ObstacleArray for a given CenterPoint - a vector with
        %3 coordinates, first two defining is position in 3D
        function ObstacleArray = GenerateObstacleArrayForGivenCenterPoint(obj, CenterPoint)
            ObstacleArray = obj.HeightMap2Obstacles(CenterPoint(3));
            ObstacleArray = [obj.ObstaclePointArray; ObstacleArray];
        end
        
        %produces a linear  inequality representation of an obstacle-free
        %region cenetred at CenterPoint
        function [A, b] = GetFreeConvexHullInequalities(obj, CenterPoint, ObstacleArray)
            if nargin < 3
                ObstacleArray = [];
            end
            if isempty(ObstacleArray)
                ObstacleArray = obj.GenerateObstacleArrayForGivenCenterPoint(CenterPoint);
            end
            
            [A, b] = obj.StereographicProjectionFreeRegionsObj.GetFreeConvexHullInequalities(ObstacleArray, CenterPoint(1:2, 1));
            % [A, b] = obj.StereographicProjectionFreeRegionsObj.GetFreeConvexHullEstimateInequalities(ObstacleArray, CenterPoint(1:2, 1));
        end
        
        %%%%%%%%%%%%%%%%
        %%% tools for tiling        
        
        %Tiles the plane with obstacle free regions originating from
        %CenterPointArray points
        function RegionsArray = GetPlaneTiling(obj, CenterPointArray, uniform_height)
            if nargin < 3
                uniform_height = [];
            end
            if ~isempty(uniform_height)
                ObstacleArray = obj.GenerateObstacleArrayForGivenCenterPoint(CenterPointArray(1, :)');
            else
                ObstacleArray = [];
            end
            
            N = size(CenterPointArray, 1);
            RegionsArray = cell(N, 1);
            for i = 1:N
                [A, b] = obj.GetFreeConvexHullInequalities(CenterPointArray(i, :)', ObstacleArray);
                RegionsArray{i}.A = A;
                RegionsArray{i}.b = b;
            end
        end
        
        %generetae CenterPointArray
        function CenterPointArray = GenerateCenterPointArray(obj, sqrtNumberOfPoints, Type)
            if nargin < 3
                Type = 'square';
            end
            if nargin < 2
                sqrtNumberOfPoints = 4;
            end
            
            n = sqrtNumberOfPoints;
            CenterPointArray = zeros(n*n, 3);
            
            [N, M] = size(obj.HeightMapArrays.X);
            
            switch Type
                case 'square'
                    index = 0;
                    for i = 1:n
                        for j = 1:n
                            I = floor(i*N/n); J = floor(j*M/n);
                            z = obj.HeightMapArrays.Z(I, J);
                            if ~isnan(z)
                                index = index + 1;
                                CenterPointArray(index, :) = [obj.HeightMapArrays.X(I, J), obj.HeightMapArrays.Y(I, J), z];
                            end
                        end
                    end
                    CenterPointArray = CenterPointArray(1:index, :);
                    
                case 'sobol'
                    warning('not implemented yet');
            end
        end
        
        %%%%%%%%%%%%%%%%
        %%% graphics
        
        %sets the figure for drawing
        function SetFigure(obj)
           if isempty(obj.drawing_figure)
               obj.drawing_figure = figure();
           else
               figure(obj.drawing_figure);
           end
           
           obj.drawing_figure.Color = 'w';
        end
        
        %draws the Height Map using HeightMapArrays
        function output = DrawHeightMapArrays(obj)
            obj.SetFigure;
            output.surf_handle = surf(obj.HeightMapArrays.X, obj.HeightMapArrays.Y, obj.HeightMapArrays.Z, ...
                'EdgeAlpha', 0.3);
            for i = 1:size(obj.LightPoisitionArray, 1)
                output.light_handle(i) = light('Position', obj.LightPoisitionArray(i, :), 'Style', 'local');
            end
            axis equal;
            hold on;
        end
        
        %draws obstacles
        function h = DrawObstacles(obj, ObstacleArray)
            if nargin < 2
                ObstacleArray = obj.ObstaclePointArray;
            end
            
            N = size(ObstacleArray, 1);
            Z = obj.drawing_obstacles_height * ones(N, 1);
            
            obj.SetFigure;
            h = plot3(ObstacleArray(:, 1), ObstacleArray(:, 2), Z, '*');
            axis equal;
            hold on;
        end
        
        %Visualizes the work of GetFreeConvexHullInequalities method
        function output = VisualizeGetFreeConvexHullInequalities(obj, CenterPoint)
            tic;
            ObstacleArray = obj.GenerateObstacleArrayForGivenCenterPoint(CenterPoint);
            [A, b] = obj.GetFreeConvexHullInequalities(CenterPoint, ObstacleArray);
            toc;
            
            obj.SetFigure;
            output.polygon_handle = obj.VisualizeInequalities(A, b);
            output.obstacles_handle = obj.DrawObstacles(ObstacleArray);
            output.CenterPoint_handle = plot3(CenterPoint(1), CenterPoint(2), obj.drawing_obstacles_height, ...
                'o', 'MarkerFaceColor', 'r');
            axis equal;
        end
        
        %Visualizes an inequality
        function h = VisualizeInequalities(obj, A, b)
            V = con2vert(A, b);
            Vertices = obj.Math.convhull(V);
            h = fill(Vertices(:, 1), Vertices(:, 2), 'y'); hold on; grid on;
        end
        
        %Visualizes the work of GetPlaneTiling method
        function output = VisualizeGetPlaneTiling(obj, RegionsArray, CenterPointArray, uniform_height)
            if nargin < 2
                RegionsArray = [];
            end
            if isempty(RegionsArray)
                if nargin < 4
                    uniform_height = [];
                end
                if nargin < 3
                    CenterPointArray = obj.GenerateCenterPointArray();
                end
                tic;
                RegionsArray = obj.GetPlaneTiling(CenterPointArray, uniform_height);
                toc;
            end
            
            N = size(RegionsArray, 1);
            obj.SetFigure;
            for i = 1:N
                output.polygon_handles(i) = obj.VisualizeInequalities(RegionsArray{i}.A, RegionsArray{i}.b);
            end
        end
        
    end
end