%This class implements some animation functionality for chains
%defined by ChainClass
%last update 03.06.16
classdef SRDAnimation < handle
    properties
        %%%%%%%%%%%%%%%%%%%
        %%%% Drawing parameters
        
        %If you need to specify a figure for drawing, use this property
        AnimationFigure = [];
        
        %Determines the axis limits
        AxisLimits = [];
        
        %Determines the view that would be used by the system
        ViewAngle = [];
        
        %Determines robot's color;
        RobotColor = [1 0 0];
        
        %%%%%%%%%%%%%%%%%%%
        %%%% Animation parameters
        
        %This property determines the time step for animation
        Animation_TimeStep = 0.02;
        
        %A constant that can be used to vary the animation speed
        Animation_Accelerator = 1;
        
        %%%%%%%%%%%%%%%%%%%
        %%%% Position Sequence
        
        %This property determines the amount of time the 
        PositionSequence_TimeStep = 0.2;        
        
        %%%%%%%%%%%%%%%%%%%
        %%%% drawing the robot
        
        %This property determines what method is used for drawing
        DrawType = 'Default';
        
        %function handle for custom robot drawing function
        DrawCustom;
        
        %fill3 parameters
        EdgeAlpha = 1;
        FaceAlpha = 1;
        LineWidth = 1;
        
        %%%%%%%%%%%%%%%%%%%
        %%%% objects of SRD classes;
        
        % SRDAnimation relies on a few other SRD classes, so it will load
        % them using SRDuserinterface loader functions unless the class  
        % constructor received a parameter ManualSetup = true;
        
        SRDuserinterface_object;
        SimulationEngine;
    end
    methods
        
        %Class constructor
        function obj = SRDAnimation(ManualSetup)
            if nargin < 1
                ManualSetup = false;
            end
            
            if ~ManualSetup
                obj.SRDuserinterface_object = SRDuserinterface();
                
                obj.AxisLimits = obj.SRDuserinterface_object.GetAxisLimits();
                obj.ViewAngle = obj.SRDuserinterface_object.GetViewAngle();
                obj.SimulationEngine = obj.SRDuserinterface_object.GetSimulationEngine();
            end
        end
        
        %This function draws the robot. The method which it uses depends on
        %DrawType property
        function h = Draw(obj, q, old_h)
            switch obj.DrawType
                case 'Default'
                    h = obj.DrawDefault(q, old_h);
                case 'Custom'
                    h = obj.DrawCustom(q, old_h);
%                 case 'STL'
%                     h = obj.DrawSTL(q, old_h);
                otherwise
                    warning('Invalid type');
            end
        end
        
        %this function draws state of the robot defined by q
        function h = DrawDefault(obj, q, old_h)
            obj.SimulationEngine.Update(q);
            h = obj.DrawCurrentPositionDefault(old_h);
        end
        
        function h = DrawCurrentPositionSTL(obj, old_h)
            if isempty(obj.AnimationFigure)
                obj.AnimationFigure = figure;
            else
                figure(obj.AnimationFigure);
            end
            hold on; grid on;
            
            n = size(obj.SimulationEngine.LinkArray, 1);
            index = 0;
            
            if isempty(old_h)
                h = cell(n, 1);
            else
                h = old_h;
            end
            camlight('headlight');
            material('dull');
            for i = 1:n
                if obj.SimulationEngine.LinkArray(i).Order > 0
                    index = index + 1;
                    if isempty(obj.SimulationEngine.LinkArray(i).StlPath)
                        continue
                    end
                        obj.SimulationEngine.LinkArray(i).Name
                        v=eye(3)/3;
                        translation = obj.SimulationEngine.LinkArray(i).AbsoluteBase;
                        v=obj.SimulationEngine.LinkArray(i).AbsoluteOrientation*v+translation';
                        plot3([translation(1),v(1,1)],[translation(2),v(1,2)],[translation(3),v(1,3)],'r');
                        plot3([translation(1),v(2,1)],[translation(2),v(2,2)],[translation(3),v(2,3)],'r');
                        plot3([translation(1),v(3,1)],[translation(2),v(3,2)],[translation(3),v(3,3)],'r');
                        
                    fv = stlread(obj.SimulationEngine.LinkArray(i).StlPath);
                    fv.vertices=fv.vertices*obj.SimulationEngine.LinkArray(i).AbsoluteOrientation+obj.SimulationEngine.LinkArray(i).AbsoluteBase';

                    patch(fv,'FaceColor',       [0.8 0.8 1.0], ...
                             'EdgeColor',       'none',        ...
                             'FaceLighting',    'gouraud',     ...
                             'AmbientStrength', 0.15);
                end
            end

        end
        
        %this function draws the current state of the robot SR
        function h = DrawCurrentPositionDefault(obj, old_h)
            if isempty(obj.AnimationFigure)
                obj.AnimationFigure = figure;
            else
                figure(obj.AnimationFigure);
            end
            hold on; grid on;
            
            n = size(obj.SimulationEngine.LinkArray, 1);
            index = 0;
            
            if isempty(old_h)
                h = cell(n, 1);
            else
                h = old_h;
            end
            for i = 1:n
                if obj.SimulationEngine.LinkArray(i).Order > 0
                    index = index + 1;
                    
                    Polygon = [obj.SimulationEngine.LinkArray(i).AbsoluteBase, obj.SimulationEngine.LinkArray(i).AbsoluteFollower];
                    
                    if isempty(old_h)
                        X = Polygon(1, :)';
                        Y = Polygon(2, :)';
                        Z = Polygon(3, :)';
                        h{i} = fill3(X, Y, Z, obj.RobotColor, 'EdgeAlpha', obj.EdgeAlpha, 'FaceAlpha', obj.FaceAlpha, ...
                            'LineWidth', obj.LineWidth);
                    else
                        h{i}.Vertices = Polygon';
                    end
                end
            end
        end
        
        %This function draws the initial position of the robot
        function h = DrawIC(obj, old_h)
            
            if nargin < 2
                old_h = [];
            end
            
            obj.SimulationEngine.Update(obj.SimulationEngine.IC.q);
            h = obj.DrawCurrentPositionSTL(old_h);
            
            axis equal;
            axis(obj.AxisLimits);
            
            %set the view
            if isempty(obj.ViewAngle)
                view(3);
            else
                view(obj.ViewAngle);
            end
        end        
        
        
        function SetFigure(obj)
            if isempty(obj.AnimationFigure)
                obj.AnimationFigure = figure;
                obj.AnimationFigure.Color = 'w';
            else
                figure(obj.AnimationFigure);
            end
            hold on; grid on; 
        end
        
        % This function makes an animation of robot's motion.
        function Animate(obj, PositionArray)
            obj.SetFigure();
            
            %set the view
            if isempty(obj.ViewAngle)
                view(3);
            else
                view(obj.ViewAngle)
            end
            
            if obj.Animation_TimeStep < obj.SimulationEngine.TimeStep
                Step = 1;
            else
                Step = obj.Animation_TimeStep / obj.SimulationEngine.TimeStep;
            end
            
            h = obj.DrawIC();
            %generate index array; the idea is the have an array that
            %includes both the beginning and the end of the position array
            Count = floor(size(PositionArray, 1) / Step);
            index = zeros(Count + 1, 1);
            for i = 1:Count
                index(i) = floor((i - 1) * Step + 1);
            end    
            index(end) = size(PositionArray, 1);
            
            %animation
            for i = 1:(Count + 1)
                q = PositionArray(index(i), :)';
                [az, el] = view; %remember the user-set view
                
                h = obj.Draw(q, h); %draw new frame
                view([az, el]); %restore the user-set view
                axis equal;
                axis(obj.AxisLimits);
                
                drawnow;
                pause(obj.Animation_TimeStep / obj.Animation_Accelerator);
            end
        end        
        
        % This function makes an animation of robot's motion.
        function PositionSequence(obj, PositionArray)
            obj.SetFigure();
            
            %set the view
            if isempty(obj.ViewAngle)
                view(3);
            else
                view(obj.ViewAngle)
            end
            
            if obj.PositionSequence_TimeStep < obj.SimulationEngine.TimeStep
                Step = 1;
            else
                Step = obj.PositionSequence_TimeStep / obj.SimulationEngine.TimeStep;
            end
            
            %generate index array; the idea is the have an array that
            %includes both the beginning and the end of the position array
            Count = floor(size(PositionArray, 1) / Step);
            index = zeros(Count + 1, 1);
            for i = 1:Count
                index(i) = floor((i - 1) * Step + 1);
            end    
            index(end) = size(PositionArray, 1);
            
            %image sequence
            for i = 1:(Count + 1)
                q = PositionArray(index(i), :)';
                [az, el] = view; %remember the user-set view
                
                obj.Draw(q, []); %draw new frame
                view([az, el]); %restore the user-set view
                axis equal;
                axis(obj.AxisLimits);
                
                drawnow;
            end
        end 
        
        
    end
end