classdef IPCenterlineBuilder < handle
    properties
        Segments;
        
        Speed = 1;
        
        Math;
    end
    methods
        
        function obj = IPCenterlineBuilder()
            obj.Math = MathClass;
        end
        
        %creates center line's first segment
        function CreateCenterline(obj, Point, Direction)
            if nargin < 2
                Point = [0; 0; 0];
            end
            
            if nargin < 3
                Direction = [1; 0; 0];
            else
                Direction = Direction / norm(Direction);
            end
            
            obj.Segments = [];
            obj.Segments{1}.Type = 'Start';
            obj.Segments{1}.R_Start = Point;
            obj.Segments{1}.R_End = Point;
            obj.Segments{1}.S_Start = 0;
            obj.Segments{1}.S_End = 0;
            obj.Segments{1}.StartDirection = Direction;
            obj.Segments{1}.EndDirection = Direction;
        end
        
        %adds straight line facing the same direction as the previous
        %segment's end
        function AddStraightSegment(obj, Length)
            Index = max(size(obj.Segments));
            R_Start = obj.Segments{Index}.R_End;
            S_Start = obj.Segments{Index}.S_End;
            Direction = obj.Segments{Index}.EndDirection;
            
            obj.Segments{Index + 1}.Type = 'Line';
            obj.Segments{Index + 1}.R_Start = R_Start;
            obj.Segments{Index + 1}.R_End = R_Start + Length*Direction;
            obj.Segments{Index + 1}.S_Start = S_Start;
            obj.Segments{Index + 1}.S_End = S_Start + Length/obj.Speed;
            obj.Segments{Index + 1}.StartDirection = Direction;
            obj.Segments{Index + 1}.EndDirection = Direction;
        end
        
        %adds U turn
        function AddUturn(obj, Axis, Radius)
            
            Index = max(size(obj.Segments));
            R_Start = obj.Segments{Index}.R_End;
            S_Start = obj.Segments{Index}.S_End;
            Direction = obj.Segments{Index}.EndDirection;
            
            %find vector pointing towards the center of rotation
            V = cross(Axis, Direction);
            V = V / norm(V);
            
            %find the center of rotation
            Center = R_Start + Radius*V;
            R_End = R_Start + 2*Radius*V;
            
            Length = pi*Radius;
            
            obj.Segments{Index + 1}.Type = 'Uturn';
            obj.Segments{Index + 1}.Center = Center;
            obj.Segments{Index + 1}.Axis = Axis;
            obj.Segments{Index + 1}.Radius = Radius;
            obj.Segments{Index + 1}.V = V;
            
            obj.Segments{Index + 1}.R_Start = R_Start;
            obj.Segments{Index + 1}.R_End = R_End;
            obj.Segments{Index + 1}.S_Start = S_Start;
            obj.Segments{Index + 1}.S_End = S_Start + Length/obj.Speed;
            obj.Segments{Index + 1}.StartDirection = Direction;
            obj.Segments{Index + 1}.EndDirection = -Direction;
        end
        
        
        %adds L turn
        function AddLturn(obj, Axis, Radius)
            
            Index = max(size(obj.Segments));
            R_Start = obj.Segments{Index}.R_End;
            S_Start = obj.Segments{Index}.S_End;
            Direction = obj.Segments{Index}.EndDirection;
            
            %find vector pointing towards the center of rotation
            V1 = cross(Axis, Direction);
            V1 = V1 / norm(V1);
            
            %find the center of rotation
            Center = R_Start + Radius*V1;
            
            %find the rotation matrix
            T = obj.Math.RotationAroundAxis(Axis, pi/2);
            
            %find vector that points from the center of rotation to the end
            %of the L turn
            V2 = -T*V1;
            V2 = V2 / norm(V2);
            
            R_End = Center + Radius*V2;
            
            EndDirection = T*Direction;
            
            Length = 0.5*pi*Radius;
            
            obj.Segments{Index + 1}.Type = 'Lturn';
            obj.Segments{Index + 1}.Center = Center;
            obj.Segments{Index + 1}.Axis = Axis;
            obj.Segments{Index + 1}.Radius = Radius;
            obj.Segments{Index + 1}.V1 = V1;
            obj.Segments{Index + 1}.V2 = V2;
            
            obj.Segments{Index + 1}.R_Start = R_Start;
            obj.Segments{Index + 1}.R_End = R_End;
            obj.Segments{Index + 1}.S_Start = S_Start;
            obj.Segments{Index + 1}.S_End = S_Start + Length/obj.Speed;
            obj.Segments{Index + 1}.StartDirection = Direction;
            obj.Segments{Index + 1}.EndDirection = EndDirection;
        end
        
        %gives the index of the segment the s belongs to (decided by
        %properties S_Start and S_End of the segments)
        function Index = RetrieveSegmentIndex(obj, s)
            Index = -1;
            NoS = max(size(obj.Segments));
            for i = 1:NoS
                s1 = obj.Segments{i}.S_Start;
                s2 = obj.Segments{i}.S_End;
                if ((s >= s1) && (s <= s2))
                    Index = i;
                end
            end
        end
        
        %evaluates the line for the current s
        function r = Evaluate(obj, s)
            Index = obj.RetrieveSegmentIndex(s);
            
            if Index == -1
                warning(['s, ', num2str(s), ' out of range']);
                Index = max(size(obj.Segments));
                [~, s] = obj.GetSrange();
            end
            
            Type = obj.Segments{Index}.Type;
            
            switch Type
                case 'Line'
                    r = obj.EvaluateLine(obj.Segments{Index}, s);
                case 'Uturn'
                    r = obj.EvaluateUturn(obj.Segments{Index}, s);
                case 'Lturn'
                    r = obj.EvaluateLturn(obj.Segments{Index}, s);
                otherwise
                    r = [NaN; NaN; NaN];
            end
        end
        
        %evaluates line segments 
        function r = EvaluateLine(~, Segment, s)
            local_s = s - Segment.S_Start;
            p = local_s / (Segment.S_End - Segment.S_Start);
            r = Segment.R_Start*(1 - p) + Segment.R_End*p;
        end
        
        %evaluates Uturn segments 
        function r = EvaluateUturn(obj, Segment, s)
            local_s = s - Segment.S_Start;
            phi = pi*local_s / (Segment.S_End - Segment.S_Start);
            
            %find the rotation matrix
            T = obj.Math.RotationAroundAxis(Segment.Axis, phi);
            
            R = Segment.R_Start - Segment.Center;
            r = Segment.Center + T*R;
        end
        
        %evaluates Lturn segments 
        function r = EvaluateLturn(obj, Segment, s)
            local_s = s - Segment.S_Start;
            phi = 0.5*pi*local_s / (Segment.S_End - Segment.S_Start);
            
            %find the rotation matrix
            T = obj.Math.RotationAroundAxis(Segment.Axis, phi);
            
            R = Segment.R_Start - Segment.Center;
            r = Segment.Center + T*R;
        end
        
        %provides the range of s for this centerline
        function [s1, s2] = GetSrange(obj)
            s1 = obj.Segments{1}.S_Start;
            s2 = obj.Segments{max(size(obj.Segments))}.S_End;
        end
        
        
    end
end