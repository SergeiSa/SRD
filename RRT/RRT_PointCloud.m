classdef RRT_PointCloud < handle
    properties
        PointCloud = [];
        %point cloud used in calculations
        
        raw_path = [];
        %raw path found by RRT algorithm
        
        processed_path = [];
        %the path found by RRT algorithm and processed (padded, smoothed,
        %...) after
        
        %%%%%%%%%%%%%%%%%%%%%
        %RRT parameters:
        
        max_number_of_RRT_nodes = 500;
        %max number of nodes in the RRT graph
        
        max_attempts_to_grow_new_node = 2000;
        %max number of attempts to grow new node for teh RRT tree
        
        distance_function = [];
        %handle to a function used to calculate distance to points
        
        plot_RRT_grows = false;
        %plot RRT graph while it is growing
        
        %%%%%%%%%%%%%%%%%%%%%
        %postprocessing parammeters:
        
        postprocessing__number_of_padding_points = 5;
        %number of points added between every two points on the found raw 
        %RRT path
        
        postprocessing__smoothing_coef = 0.5;
        %coefficient between 0 and 1, determining the smoothig rate for
        %smoothing postprocessing
        
        postprocessing__smoothing_number_of_iterations = 10;
        %number of times smoothing is applied
        
        smoothing_function = [];
        %handle to a function used to smooth the path
        
        %%%%%%%%%%%%%%%%%%%%%
        %reachability functions parameters:
        
        pipe_reachability_NumberOfPoints = 10;
        %how many points to check between the closest RRT node and the new
        %node
        
        pipe_reachability_DivergenceLimit = 0.1;
        %how much the line between the closest RRT node and the new node
        %can diverge from the point cloud
        
        %%%%%%%%%%%%%%%%%%%%%
        %general parameters
        
        verbose = true;
        %if true - reports will be printed
        
    end
    methods
        function obj = RRT_PointCloud(PointCloud)
            obj.PointCloud = PointCloud;
            
            obj.distance_function = @obj.pipe_distance;
            obj.smoothing_function = @obj.smooth_path2;
        end
        
        function status = FindPathAndProcess(obj, Start, Goal)
            %find path
            status = FindPath(obj, Start, Goal);
            
            if status == 1
                %pad path
                [padded_path, padded_path_indices] = obj.pad_path(obj.raw_path.path, obj.raw_path.path_indices);
                obj.processed_path.padded_path = padded_path;
                obj.processed_path.padded_path_indices = padded_path_indices;
                
                %smooth path
                [smoothed_path, smoothed_path_indices] = obj.smoothing_function(padded_path, padded_path_indices);
                obj.processed_path.path = smoothed_path;
                obj.processed_path.path_indices = smoothed_path_indices;
            end
        end
        
        %solves RRT problem
        function status = FindPath(obj, Start, Goal)
            
            %find starting and the ending poinst of the path
            obj.raw_path.Start.index = dsearchn(obj.PointCloud, Start');
            obj.raw_path.Goal.index = dsearchn(obj.PointCloud, Goal');
            
            obj.raw_path.Start.point = obj.PointCloud(obj.raw_path.Start.index, :);
            obj.raw_path.Goal.point = obj.PointCloud(obj.raw_path.Goal.index, :);
            
            MaxIndex = size(obj.PointCloud, 1);
            
            RRT_IndexArray = zeros(obj.max_number_of_RRT_nodes, 2);
            RRT_PointArray = NaN(obj.max_number_of_RRT_nodes, 3);
            
            RRT_IndexArray(1, :) = [obj.raw_path.Start.index, 0];
            RRT_PointArray(1, :) = obj.raw_path.Start.point;
            
            i = 2; status = 0;
            while (i < obj.max_number_of_RRT_nodes) && (status == 0)
                j = 1; not_found = true;
                while (j < obj.max_attempts_to_grow_new_node) && not_found
                    %pick randon index / random point from the cloud
                    NewNode_index = randi([1, MaxIndex]);
                    NewNode_point = obj.PointCloud(NewNode_index, :);
                    
                    if isfinite(NewNode_point)
                        optimal_node_index_RRT = dsearchn(RRT_PointArray, NewNode_point);
                        optimal_node           = RRT_PointArray(optimal_node_index_RRT, :);
                        
                        distance = obj.distance_function(optimal_node, NewNode_point);
                    else
                        distance = Inf;
                    end
                    
                    %check if the closes point in the tree is close enough
                    if isfinite(distance)
                        not_found = false;
                        
                        RRT_IndexArray(i, :) = [NewNode_index, optimal_node_index_RRT];
                        RRT_PointArray(i, :) = NewNode_point;
                        
                        if obj.plot_RRT_grows
                            Line = [optimal_node; NewNode_point];
                            plot3(Line(:, 1), Line(:, 2), Line(:, 3), 'LineWidth', 1, 'Color', 'k'); hold on;
                            drawnow;
                        end
                    end
                    j = j + 1;
                    if j >= obj.max_number_of_RRT_nodes
                        warrning('Can not make progress');
                        status = -1;
                    end
                end
                
                if status >= 0
                    %check if goal is within reach
                    distance = obj.distance_function(obj.raw_path.Goal.point, NewNode_point);
                    if isfinite(distance)
                        i = i + 1;
                        RRT_IndexArray(i, :) = [obj.raw_path.Goal.index, i-1];
                        RRT_PointArray(i, :) =  obj.raw_path.Goal.point;
                        
                        RRT_IndexArray = RRT_IndexArray(1:i, :);
                        RRT_PointArray = RRT_PointArray(1:i, :);
                        status = 1;
                    end
                    i = i + 1;
                end
            end
            
            k = size(RRT_IndexArray, 1);
            found_path = []; found_path_indices = [];
            if status >= 0
                while k ~= 0
                    PathIndex = RRT_IndexArray(k, 1);
                    PathNode = obj.PointCloud(PathIndex, :);
                    
                    found_path = [found_path; PathNode];
                    found_path_indices = [found_path_indices; PathIndex];
                    k = RRT_IndexArray(k, 2);
                end
            end
            
            if obj.verbose
                if status == 1
                    disp(['found path with ', num2str(size(found_path, 1)), ' nodes']);
                else
                    disp('path not found');
                end
            end
            
            obj.raw_path.RRT_IndexArray = RRT_IndexArray;
            obj.raw_path.RRT_PointArray = RRT_PointArray;
            obj.raw_path.path = found_path;
            obj.raw_path.path_indices = found_path_indices;
        end
        
        %pads the path
        function [padded_path, padded_path_indices] = pad_path(obj, path, path_indices)
            
            NoP = obj.postprocessing__number_of_padding_points + 1;
            
            n = size(path, 1);
            m = (n-1) * NoP;
            
            padded_path = zeros(m, 3);
            padded_path_indices = zeros(m, 1);
            
            for i = 1:(n-1)
                P1 = path(i, :);
                P2 = path(i+1, :);
                
                for j = 1:NoP
                    index = (i - 1) * NoP + j;
                    P = P1 + (P2-P1) * (j / NoP);
                    
                    closest_index = dsearchn(obj.PointCloud, P);
                    P_permissible = obj.PointCloud(closest_index, :);
                    
                    padded_path(index, :) = P_permissible;
                    padded_path_indices(index) = closest_index;
                end
            end
            padded_path = [path(1, :); padded_path];
            padded_path_indices = [path_indices(1); padded_path_indices];
                               
            if obj.verbose
               disp(['path padded, from ', num2str(size(path, 1)), ' nodes to ', num2str(size(padded_path, 1))]);
            end
        end
        
        %smoothes the path
        function [smoothed_path, smoothed_path_indices] = smooth_path(obj, path, path_indices)
            
            mu = obj.postprocessing__smoothing_coef;
            
            smoothed_path = path;
            smoothed_path_indices = path_indices;
            n = size(smoothed_path, 1);
            
            for k = 1:obj.postprocessing__smoothing_number_of_iterations
                for i = 2:(n-1)
                    P1 = smoothed_path(i-1, :);
                    P2 = smoothed_path(i, :);
                    P3 = smoothed_path(i+1, :);
                    
                    Pmean = (P1 + P3) / 2;
                    P2_new = P2*(1 - mu) + Pmean*mu;
                    
                    closest_index = dsearchn(obj.PointCloud, P2_new);
                    P2_fixed = obj.PointCloud(closest_index, :);
                    
                    if ~isfinite(P2_fixed)
                        P2_fixed = P2;
                        closest_index = path_indices(i);
                    end
                    smoothed_path(i, :) = P2_fixed;
                    smoothed_path_indices(i, :) = closest_index;
                end
            end
        end
        %smoothes the path
        function [smoothed_path, smoothed_path_indices] = smooth_path2(obj, path, path_indices)
            
            mu = obj.postprocessing__smoothing_coef;
            
            smoothed_path = path;
            smoothed_path_indices = path_indices;
            N = size(smoothed_path, 1);
            
            A = zeros(N, N);
            A(1, 1) = 1; 
            A(N, N) = 1;
            A(2, 1:4) = [1/8, 3/8, 3/8, 1/8];
            A(N-1, (N-3):N) = [1/8, 3/8, 3/8, 1/8];
            for i = 3:(N-2)
                A(i, (i-2):(i+2)) = [1/16, 4/16, 6/16, 4/16, 1/16];
            end
            A = mu*A + (1 - mu)*eye(N);
            
            for k = 1:obj.postprocessing__smoothing_number_of_iterations
                
                potential_path = A * smoothed_path;
                for i = 2:(N-1)
                    closest_index = dsearchn(obj.PointCloud, potential_path(i, :));
                    P_fixed = obj.PointCloud(closest_index, :);
                    
                    if ~isfinite(P_fixed)
                        P_fixed = smoothed_path(i, :);
                        closest_index = path_indices(i);
                    end
                    smoothed_path(i, :) = P_fixed;
                    smoothed_path_indices(i, :) = closest_index;
                end
            end
        end
        
        %checks if the points between the two selected ones are lying close 
        %enough to the pipe's surface
        function distance = pipe_distance(obj, Point1, Point2)
            
            delta_point = Point2 - Point1;
            
            failed = false;
            for i = 1:obj.pipe_reachability_NumberOfPoints 
                Point = Point1 + delta_point*(i - 1) / obj.pipe_reachability_NumberOfPoints;
                Point = reshape(Point, [1, 3]);
                
                closest_index = dsearchn(obj.PointCloud, Point);
                Point_InCould = obj.PointCloud(closest_index, :);
                
                L = norm(Point-Point_InCould);
                
                if (L > obj.pipe_reachability_DivergenceLimit) || (~isfinite(L))
                    failed = true;
                end
            end
            
            if ~failed
                distance = norm(Point1 - Point2);
            else
                distance = Inf;
            end
        end
    end
end