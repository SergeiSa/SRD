%TO DO memort preallocaton for the additional inequalities
classdef SPStereographicProjectionFreeRegions < handle
    properties
        Math;
        %object of MathClass class
        
        %%%%%%%%%%%%%%%%
        %%% algorithm settings
        
        check_for_NaNs = false;
        
        %%%%%%%%%%%%%%%%
        %%% graphics
        
        figure_handle = [];
    end
    methods
        function obj = SPStereographicProjectionFreeRegions()
            obj.Math = MathClass;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Projectors
        
        %Projector onto unit sphere, 2D -> 3D
        %P is a a vector in R^2, R is a vector in R^3
        function R = Projector(~, P)
            PP = P(1)^2 + P(2)^2;
            
            R(1, 1) = 2*P(1) / (1 + PP);
            R(2, 1) = 2*P(2) / (1 + PP);
            R(3, 1) = (-1 + PP) / (1 + PP);
        end
        
        %Projector from n-space onto (n+1) unit sphere, nD -> (n+1)D
        %P is a a vector in R^n, R is a vector in R^(n + 1)
        function R = ProjectorN(~, P)
            N = max(size(P));
            R = zeros(N+1, 1);
            
            PP = dot(P, P);
            for i = 1:N
                R(i, 1) = 2*P(i) / (1 + PP);
            end
            R(N+1, 1) = (-1 + PP) / (1 + PP);
        end
     
        %Projector onto unit sphere, 2D -> 3D
        %P and R are vectors in R^2
        %
        %unlike Projector() this function does not compute the last entry
        %of the output (the height of the point), effectively projecting
        %the point onto the sphere and then back onto the plane
        function R = Projector_fast(~, P)
            R(:, 1) = 2*P / (1 + P(1)^2 + P(2)^2);
        end
        
        %Projector from n-space onto (n+1) unit sphere, nD -> (n+1)D
        %P and R are vectors in R^n
        %
        %unlike Projector() this function does not compute the last entry
        %of the output (the height of the point), effectively projecting
        %the point onto the sphere and then back onto the plane
        function R = ProjectorN_fast(~, P)
            N = max(size(P));
            R = zeros(N, 1);
            
            PP = dot(P, P);
            for i = 1:N
                R(i, 1) = 2*P(i) / (1 + PP);
            end
        end        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Bounders 2D
        
        %returns an estimate of the convex hull of the free space centered
        %at point Center;
        %
        %Points - array of points in 2D space, two columns of x and y
        %coordinates;
        %K - array of indices, see convhull description
        function K = GetFreeConvexHullEstimate(obj, Points, Center)
            N = size(Points, 1);
            
            CenteredPoints = Points - [ones(N, 1)*Center(1), ones(N, 1)*Center(2)];
            ProjectedPoints = zeros(N, 2);
            
            for i = 1:N
                ProjectedPoints(i, :) = obj.Projector_fast(CenteredPoints(i, :)');
            end
            
            K = convhull(ProjectedPoints);
        end        
        
        %returns an estimate of the convex hull of the free space centered
        %at point Center;
        %
        %Points - array of points in 2D space, two columns of x and y
        %coordinates;
        %K - array of indices, see convhull description
        function K = GetFreeConvexHullEstimateN(obj, Points, Center)
            [N, M] = size(Points);
            
            CenteredPoints = Points;
            for j = 1:M
                CenteredPoints(:, j) = CenteredPoints(:, j) - ones(N, 1)*Center(j);
            end
            ProjectedPoints = zeros(N, M);
            
            for i = 1:N
                ProjectedPoints(i, :) = obj.ProjectorN_fast(CenteredPoints(i, :)');
            end
            
            if M <= 3
            K = convhull(ProjectedPoints);
            else
                warning('The code for dimentions higher than 3 is not here yet')
            end
        end        
        
        %same as GetFreeConvexHullEstimate(), but returns linear inequalities
        function [A, b] = GetFreeConvexHullEstimateInequalities(obj, Points, Center)
            K = obj.GetFreeConvexHullEstimate(Points, Center);
            
            N = size(K, 1);
            Vertices = zeros(N, 2);
            for i = 1:N
                Vertices(i, :) = Points(K(i), :);
            end
            [A, b] = vert2con(Vertices);
        end              
     
        %same as GetFreeConvexHullEstimateN(), but returns linear inequalities
        function [A, b] = GetFreeConvexHullEstimateInequalitiesN(obj, Points, Center)
            K = obj.GetFreeConvexHullEstimateN(Points, Center);
            
            K = K(:);
            K = unique(K);
            
            N = size(K, 1);
            M = size(Points, 2);
            Vertices = zeros(N, M);
            for i = 1:N
                Vertices(i, :) = Points(K(i), :);
            end
            [A, b] = vert2con(Vertices);
        end          
        
        %adds new inequalities to keep all points in PointsInside array out
        %of the region defined by the inequalities
        function [A, b] = Get_additional_inequalities(~, PointsInside, Center, oldA, oldb)
            
            if nargin < 3
                oldA = [];
            end
            
            if nargin < 4
                oldb = [];
            end
            
            A = oldA; b = oldb;
            
            local_N = size(PointsInside, 1);
            for local_i = 1:local_N
                
                p = PointsInside(local_i, :)' - Center;
                
                point_already_outside = false;
                if ~isempty(A)
                    if A*p > b
                        point_already_outside = true;
                    end
                end
                
                if ~point_already_outside
                    norm_p = norm(p);
                    
                    b = [b; (norm_p + dot(p, Center) / norm_p)];
                    A = [A; (p / norm_p)'];
                end
            end
        end
        
        %returns the convex hull of the free space centered at point
        %Center;
        %
        %Points - array of points in 2D space, two columns of x and y
        %coordinates;
        function [A, b] = GetFreeConvexHullInequalities(obj, Points, Center, ToTrimm)
            if nargin < 4
                ToTrimm = true;
            end
            [A, b] = obj.GetFreeConvexHullEstimateInequalities(Points, Center);
            PointsInside = obj.GetPointsInside(Points, A, b);
            [A, b] = obj.Get_additional_inequalities(PointsInside, Center, A, b);
            if ToTrimm
                [A, b] = obj.TrimmLinearInequalities(A, b);
            end
        end
        
        %returns the convex hull of the free space in R^n centered at point
        %Center;
        %
        %Points - array of points in R^n space, n columns;
        function [A, b] = GetFreeConvexHullInequalitiesN(obj, Points, Center)
            if nargin < 4
                ToTrimm = true;
            end
            [A, b] = obj.GetFreeConvexHullEstimateInequalitiesN(Points, Center);
            PointsInside = obj.GetPointsInside(Points, A, b);
            [A, b] = obj.Get_additional_inequalities(PointsInside, Center, A, b);
            if ToTrimm
                [A, b] = obj.TrimmLinearInequalities(A, b);
            end
        end
        
        %deletes redundant inequality constraints
        function [A, b] = TrimmLinearInequalities(obj, A, b)
            if obj.check_for_NaNs
                LIA = ~isnan(A);
                LIV = LIA(:, 1) | LIA(:, 2);
                
                A = A(LIV, :);
                b = b(LIV);
            end
            
            [~, nr] = con2vert(A, b);
            A = A(nr, :);
            b = b(nr);
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% General tools       
        
        function PointsInside = GetPointsInside(~, Points, A, b)
            N = size(Points, 1);
            
            PointsInside = zeros(size(Points));
            index = 0;
            
            for i = 1:N
                if A*Points(i, :)' < b
                    index = index + 1;
                    PointsInside(index, :) = Points(i, :);
                end
            end
            
            PointsInside = PointsInside(1:index, :);
        end        
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Visuals
        
        %Sets up active figure for drawing
        function SetUpFigure(obj)
            if isempty(obj.figure_handle)
                obj.figure_handle = figure();
            else
                figure(obj.figure_handle);
            end
            
            obj.figure_handle.Color = 'w';
        end
        
        %Visualizes the work of GetFreeConvexHullEstimate method
        function VisualizeFreeConvexHullEstimate(obj, Points, Center)
            obj.SetUpFigure;
            tic;
            K = obj.GetFreeConvexHullEstimate(Points, Center);
            toc;
            
            N = size(K, 1);
            Vertices = zeros(N, 2);
            for i = 1:N
                Vertices(i, :) = Points(K(i), :);
            end
            fill(Vertices(:, 1), Vertices(:, 2), 'y'); hold on; grid on;
            plot(Points(:, 1), Points(:, 2), '*');
            plot(Center(1), Center(2), 'o', 'MarkerFaceColor', 'r');
            axis equal;
        end
        
        %Visualizes the work of GetFreeConvexHullEstimateN method for the
        %3D input
        function VisualizeFreeConvexHullEstimate3D(obj, Points, Center)
            obj.SetUpFigure;
            tic;
            K = obj.GetFreeConvexHullEstimateN(Points, Center);
            toc;
            
            trisurf(K, Points(:, 1), Points(:, 2), Points(:, 3)); hold on; grid on;
            
            plot3(Points(:, 1), Points(:, 2), Points(:, 3), '*');
            plot3(Center(1), Center(2), Center(3), 'o', 'MarkerFaceColor', 'r');
            axis equal;
        end
        
        
        %Visualizes the work of GetFreeConvexHullInequalities method
        function VisualizeGetFreeConvexHullInequalities(obj, Points, Center)
            tic;
            [A, b] = GetFreeConvexHullInequalities(obj, Points, Center);
            toc;
            
            V = con2vert(A, b);
            Vertices = obj.Math.convhull(V);
            
            obj.SetUpFigure;
            fill(Vertices(:, 1), Vertices(:, 2), 'y'); hold on; grid on;
            plot(Points(:, 1), Points(:, 2), '*');
            plot(Center(1), Center(2), 'o', 'MarkerFaceColor', 'r');
            axis equal;
        end
        
        %Visualizes the work of GetFreeConvexHullInequalitiesN method
        function VisualizeGetFreeConvexHullInequalities3D(obj, Points, Center)
            tic;
            [A, b] = GetFreeConvexHullInequalitiesN(obj, Points, Center);
            toc;
            
            V = con2vert(A, b);
            K = convhull(V);
            
            obj.SetUpFigure;
            trisurf(K, V(:, 1), V(:, 2), V(:, 3), 'FaceAlpha', 0.5); hold on; grid on;
            
            plot3(Points(:, 1), Points(:, 2), Points(:, 3), '*');
            plot3(Center(1), Center(2), Center(3), 'o', 'MarkerFaceColor', 'r');
            axis equal;
        end
        
    end
end
