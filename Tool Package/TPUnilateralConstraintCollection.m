classdef TPUnilateralConstraintCollection < handle
    properties
        
    end
    methods
        %generates functions that represent constraints in 3D Euclidean
        %space; supports constraints given as half planes;
        %
        %generates two functions, intersection_tester()
        %get_constraint_properties() - their handles are returned as
        %fields of the output;
        %
        %All half planes should be defined as a*x < b, a is 1x3, b is a 
        %scalar, cell_a{i} = a, cell_b{i} = b
        %
        % example:
        % cell_a{1} = [0, 0, 1]; cell_b{1} = 0;
        % output = obj.HalfPlanesCell(cell_a, cell_b);
        function output = HalfPlanesCell(~, cell_a, cell_b)
            number_of_half_planes = length(cell_a);
            
            InputsAreCorrect = true;
            for i = 1:number_of_half_planes
                if (size(cell_a{i}, 1) ~= 1) || (size(cell_a{i}, 2) ~= 3) || ...
                        (size(cell_b{i}, 1) ~= 1) || (size(cell_b{i}, 2) ~= 1)
                    InputsAreCorrect = false;
                end
            end
            if ~InputsAreCorrect
                error(['All half planes should be defined as a*x < b, a is 1x3, b is a scalar; ', ...
                    'cell_a contains a, cell_b contains b']);
            end
            
            %tests whether or not the point breaks a constraint - 
            %a*x >= b for any half plain; x = Point
            function intersection = intersection_tester(Point)
                intersection = false;
                
                for local_i = 1:number_of_half_planes
                    if cell_a{local_i} * Point >= cell_b{local_i}
                        intersection = true;
                    end
                end
            end
            
            %finds info about the violated constraint; the info contains: 
            % a point C on the boundary of the half plane closest to the input Point; 
            % normal to the half plane;
            % constraint penetration depth.
            %
            %This works as follows. We find penetration depth for each
            %violated constraint and then choose the biggest one, giving
            %the info about this contstraint as the output.
            %To find penetration depth we first find n = a/||a||, the
            %negative normal for the half plane.
            %Second, we find point rO, where the line, defined by a 
            %intersects the half plane boundary: rO = b*a;
            %Third, we project Point onto null space of n, to get rCO, the
            %component of Point that describes how far Point's projection  
            %onto the half plane's boundary is from rO:
            %rOC = Point - dot(Point, n)*n;
            %Finally, we find the point C on the boundary that is closest to
            %Point: rC = rO + rOC;
            %
            %We also find basis V in the null space of n. We call it a
            %tangent basis, in a general case it wuld be a basis in the
            %plane, tangent to the surface at point of contact. Here it is
            %a bases on the boundary;
            function Output = get_constraint_properties(Point)
                
                index = 0;
                for local_i = 1:number_of_half_planes
                    if cell_a{local_i} * Point >= cell_b{local_i}
                        index = index + 1;
                        a = cell_a{local_i}; b = cell_b{local_i};
                        n = (a') / norm(a);
                        rO = b*(a');
                        rOC = Point - dot(Point, n)*n;
                        rC = rO + rOC;
                        depth = norm(Point - rC);
                        
                        V = null(n');
                        
                        PenetrationDepth(index) = depth;
                        
                        Constraints{index}.Normal = -n;
                        Constraints{index}.ConstraintPoint = rC;
                        Constraints{index}.TangentBasis = V;
                    end
                end               
                
                [~, I] = max(PenetrationDepth);
                
                Output = Constraints{I};
            end
            
            output.intersection_tester = @intersection_tester;
            output.get_constraint_properties = @get_constraint_properties;
        end
        
        %generates functions that represent constraints in 3D Euclidean
        %space as a horizontal floor;
        %
        %height - the height of the floor;
        %
        %wrapper for .HalfPlanesCell()
        function output = Floor(obj, height)
            
            cell_a{1} = [0, 0, -1];
            cell_b{1} = -height;
            
            output = obj.HalfPlanesCell(cell_a, cell_b);
        end
        
        %generates functions that represent constraints in 3D Euclidean
        %space as a inclined floor;
        %
        %height - the height of the floor;
        %T - rotation matrix
        %
        %wrapper for .HalfPlanesCell()
        function output = InclinedFloor(obj, height, T)
            
            n = [0; 0; 1];
            a = -(T*n)';
            
            cell_a{1} = a;
            cell_b{1} = -height;
            
            output = obj.HalfPlanesCell(cell_a, cell_b);
        end
        

        %generates functions that represent constraints in 3D Euclidean
        %space as a inclined floor;
        %
        %height_floor - the height of the floor;
        %height_ceiling - the height of the floor;
        %T - rotation matrix
        %
        %wrapper for .HalfPlanesCell()
        function output = Corridor(obj, height_floor, height_ceiling, T)
            
            n = [0; 0; 1];
            a = (T*n)';
            
            cell_a{1} = -a;
            cell_b{1} = -height_floor;
            cell_a{2} = a;
            cell_b{2} = height_ceiling;
            
            output = obj.HalfPlanesCell(cell_a, cell_b);
        end        
    end
end