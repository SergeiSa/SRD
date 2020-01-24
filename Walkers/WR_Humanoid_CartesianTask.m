classdef WR_Humanoid_CartesianTask < handle
    properties
        StepSequencer;
        %object of WRStepSequencer class
        
        CoM_trajectory;
        %object of WRCoM_trajectory class
        
        Splines_feet1;
        Splines_feet2;
        Splines_CoM;
        %object of TPSplineConstructorUI class
        
        %%%%%%%%%%%%%%%%%%%%%%%%%
        %%% settings
        
        NumberOfSteps = 10;
        
        StepHeight = 0.1;
        %the maximum vertical separation between the leg and the ground
        
        StepDuration = 0.5;
        %the time for a single step
        
        UseSmoothFirstOrderDerivatives = false;
        UseSmoothSecondOrderDerivatives = false;
        %if true, the spline nodes will be set to have zero value
        %derivatives
        
        %%%%%%%%%%%%%%%%%%%%%%%%%
        %%% properties
        
        TotalTime;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%
        %%% custom functions
        CustomFunction_PositionCoM = [];
        CustomFunction_OrientationTorso = [];
        CustomFunction_OrientationFoot1 = [];
        CustomFunction_OrientationFoot2 = [];
        
    end
    methods
        function obj = WR_Humanoid_CartesianTask()
            obj.Splines_feet1 = TPSplineConstructorUI();
            obj.Splines_feet2 = TPSplineConstructorUI();
            obj.Splines_CoM = TPSplineConstructorUI();
            
            obj.Splines_feet1.OutOfBoundariesBehaviour = 'LastValue';
            obj.Splines_feet2.OutOfBoundariesBehaviour = 'LastValue';
            obj.Splines_CoM.OutOfBoundariesBehaviour = 'LastValue';
        end
        
        %This function sets up the task for robot's feet
        function SetUpFootPlacementTask(obj)
            
            %ZDO stands for zero order derivative
            ZDO_Nodes_Foot1 = cell(3, (obj.NumberOfSteps*2 + 1));
            ZDO_Nodes_Foot2 = cell(3, (obj.NumberOfSteps*2 + 1));
            
            %set up the initial position of the legs
            ZDO_Nodes_Foot1(:, 1) = num2cell(obj.StepSequencer.current_positions(1, :))';
            ZDO_Nodes_Foot2(:, 1) = num2cell(obj.StepSequencer.current_positions(2, :))';
            
            for i = 1:obj.NumberOfSteps
                Output = obj.StepSequencer.MakeNextStep();
                
                index = 2*i;
                
                %node for mid-air position
                Nodes1 = (Output.StepFrom + Output.StepTo) / 2;
                Nodes1(3) = Nodes1(3) + obj.StepHeight;
                Nodes1 = num2cell(Nodes1);
                
                %node for touchdown position
                Nodes2 = Output.StepTo;
                Nodes2 = num2cell(Nodes2);
                
                if Output.LegNumber == 1
                    ZDO_Nodes_Foot1(:, index) = Nodes1;
                    ZDO_Nodes_Foot1(:, index+1) = Nodes2;
                    ZDO_Nodes_Foot2(:, index) = ZDO_Nodes_Foot2(:, index-1);
                    ZDO_Nodes_Foot2(:, index+1) = ZDO_Nodes_Foot2(:, index-1);
                else
                    ZDO_Nodes_Foot1(:, index) = ZDO_Nodes_Foot1(:, index-1);
                    ZDO_Nodes_Foot1(:, index+1) = ZDO_Nodes_Foot1(:, index-1);
                    ZDO_Nodes_Foot2(:, index) = Nodes1;
                    ZDO_Nodes_Foot2(:, index+1) = Nodes2;
                end
            end
            
            %if requested - the 1st and 2nd devivatives will be all set to
            %zero at the nodes, making the motion more "robotic", with many stops
            if obj.UseSmoothFirstOrderDerivatives
                FirstOrderDerivativeNodes = obj.Splines_feet1.GenerateEmptyArray(ZDO_Nodes_Foot1, 0);
            else
                FirstOrderDerivativeNodes = obj.Splines_feet1.GenerateEmptyArray(ZDO_Nodes_Foot1, '*');
            end
            if obj.UseSmoothSecondOrderDerivatives
                SecondOrderDerivativeNodes = obj.Splines_feet1.GenerateEmptyArray(ZDO_Nodes_Foot1, 0);
            else
                SecondOrderDerivativeNodes = obj.Splines_feet1.GenerateEmptyArray(ZDO_Nodes_Foot1, '*');
            end    
            
            NodeTimes = (0:(obj.StepDuration/2):(obj.NumberOfSteps*obj.StepDuration))';
            obj.TotalTime = NodeTimes(end);
            
            %generate splines
            obj.Splines_feet1.GenerateSplines(NodeTimes, ZDO_Nodes_Foot1, FirstOrderDerivativeNodes, SecondOrderDerivativeNodes);
            obj.Splines_feet2.GenerateSplines(NodeTimes, ZDO_Nodes_Foot2, FirstOrderDerivativeNodes, SecondOrderDerivativeNodes);
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% task evaluation funtions
        
        function Task = EvaluateTask_PositionCoM(obj, t)
            if ~isempty(obj.CustomFunction_PositionCoM)
                Task = obj.CustomFunction_PositionCoM(t);
            else
                error('Must provide a function handle CustomFunction_PositionCoM to a function evaluating the desired position of the center of mass');
            end
        end
        
        function Task = EvaluateTask_OrientationTorso(obj, t)
            if ~isempty(obj.CustomFunction_OrientationTorso)
                Task = obj.CustomFunction_OrientationTorso(t);
            else
                Task = [0; 0; 0];
            end
        end
        
        function Task = EvaluateTask_PositionFoot1(obj, t)
            Task = obj.Splines_feet1.EvaluateQ(t);
        end
        
        function Task = EvaluateTask_PositionFoot2(obj, t)
            Task = obj.Splines_feet2.EvaluateQ(t);
        end
        
        function Task = EvaluateTask_OrientationFoot1(obj, t)
            if ~isempty(obj.CustomFunction_OrientationFoot1)
                Task = obj.CustomFunction_OrientationFoot1(t);
            else
                Task = [0; 0];
            end
        end
        
        function Task = EvaluateTask_OrientationFoot2(obj, t)
            if ~isempty(obj.CustomFunction_OrientationFoot2)
                Task = obj.CustomFunction_OrientationFoot2(t);
            else
                Task = [0; 0];
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% graphics        
        
        function PlotFeetTrajectories(obj, figure_handle)
            
            if nargin < 2
                figure_handle = figure;
                figure_handle.Color = 'w';
            else
                figure(figure_handle);
            end
            
            dt = 0.01; Count = floor(obj.TotalTime / dt);
            
            TrajectoriesFoot1 = zeros(Count, 3);
            TrajectoriesFoot2 = zeros(Count, 3);
            for i = 1:Count
                t = i*dt;
                TrajectoriesFoot1(i, :) = obj.Splines_feet1.EvaluateQ(t);
                TrajectoriesFoot2(i, :) = obj.Splines_feet2.EvaluateQ(t);
            end
            
            plot3(TrajectoriesFoot1(:, 1), TrajectoriesFoot1(:, 2), TrajectoriesFoot1(:, 3), ...
                'LineWidth', 3, 'Color', 'b'); hold on; grid on;
            plot3(TrajectoriesFoot2(:, 1), TrajectoriesFoot2(:, 2), TrajectoriesFoot2(:, 3), ...
                'LineWidth', 3, 'Color', 'r');
        end
        
    end 
end