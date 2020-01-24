classdef WRCoM_trajectory < handle
    properties
        %object of WRStepSequencer class
        StepSequencer
        %object of TPSplineConstructorUI class
        Trajectory
        
        %sequence of steps that the robot will take;
        Sequence
        
        %%%%%%%%%%%%%%%
        %%% Parameters
        
        %sets the duration of a single step
        StepDuration = 0.5;
        
        %defines the position of the foot's center
        FootCenter = [0, 0, 0; 0, 0, 0];
        
        %defines the desired height of the center of mass
        CoM_height = 1;
        
        %%%%%%%%%%%%%%%
        %%% ZMP Parameters
        
        %structure with fields .Position .Velocity .Acceleration .Time
        %which are arrays containing CoM trajectory
        CoM_trajectory_arrays;
        
        
        %%%%%%%%%%%%%%%
        %%% Solver Parameters
        
        %if not empty, it will be passed to YALMIP as parameters
        Custom_YALMIP_opt = [];
        
        
    end
    methods
        function obj = WRCoM_trajectory(StepSequencer)
            obj.StepSequencer = StepSequencer;
        end
        
        %generates a sequence of steps the robot fill take, as needed for
        %center of mass trajectory planning
        function Sequence = GenerateSequence(obj, NumberOfSteps)
            
            Sequence.Position = zeros(NumberOfSteps+1, 3);
            Sequence.LegNumber = zeros(NumberOfSteps+1, 1);
            
            %find the first stance leg and add it to the sequence
            previous_sequence_position = obj.StepSequencer.current_sequence_position - 1;
            if previous_sequence_position < 1
                previous_sequence_position = 2;
            end
            index = obj.StepSequencer.sequence(previous_sequence_position);
            Sequence.Position(1, :) = obj.StepSequencer.current_positions(index, :);
            Sequence.LegNumber(1) = index;
            
            %add stance leg positions during walking
            for i = 1:NumberOfSteps
                NextStep = obj.StepSequencer.MakeNextStep();
                Sequence.Position(i+1, :) = NextStep.StepTo';
                Sequence.LegNumber(i+1) = NextStep.LegNumber;
            end
            
            if nargout < 1
                obj.Sequence = Sequence;
            end
        end
        
        %generates a row compatible with TPSplineConstructorUI class
        %the idea is to then use that class to generate a spline
        %trajectory
        function Row = GenerateRow(obj, IC)
            N = size(obj.Sequence.Position, 1);
            d = size(IC, 1);
            
            Row = cell(d, 2*N+1);
            Row(1:d, 1) = num2cell(IC);
            
            for i = 1:N
                index = 2*(i - 1) + 2;
                Row(1:d, index) = num2cell(obj.Sequence.Position(i, 1:d)' + obj.FootCenter(obj.Sequence.LegNumber(i), :)');
                Row(1:d, index+1) = num2cell(obj.Sequence.Position(i, 1:d)' + obj.FootCenter(obj.Sequence.LegNumber(i), :)');
            end
        end
        
        %generates spline trajectories; if SetDirivativesToZero == 0 then
        %the splines don't care about their derivatives, if 1 - first
        %derivatives are set to zero at the nodes, if 2 - both first and 
        %second derivatives are set to zero at the nodes.
        function GenerateSplineTrajectory(obj, IC, SetDirivativesToZero)
            
            if nargin < 3
                SetDirivativesToZero = 0;
            end
            
            obj.Trajectory = TPSplineConstructorUI();
            
            %generate spline nodes
            ZeroOrderDerivativeNodes = obj.GenerateRow(IC);
            
            switch SetDirivativesToZero
                case 0
                    FirstOrderDerivativeNodes = obj.Trajectory.GenerateEmptyArray(ZeroOrderDerivativeNodes, '*');
                    SecondOrderDerivativeNodes = obj.Trajectory.GenerateEmptyArray(ZeroOrderDerivativeNodes, '*');
                case 1
                    FirstOrderDerivativeNodes = obj.Trajectory.GenerateEmptyArray(ZeroOrderDerivativeNodes, 0);
                    SecondOrderDerivativeNodes = obj.Trajectory.GenerateEmptyArray(ZeroOrderDerivativeNodes, '*');
                case 2
                    FirstOrderDerivativeNodes = obj.Trajectory.GenerateEmptyArray(ZeroOrderDerivativeNodes, 0);
                    SecondOrderDerivativeNodes = obj.Trajectory.GenerateEmptyArray(ZeroOrderDerivativeNodes, 0);
            end
            
            NodeTimes = (0:(obj.StepDuration/2):((size(ZeroOrderDerivativeNodes, 2) - 1)*obj.StepDuration/2))';
            
            obj.Trajectory.GenerateSplines(NodeTimes, ZeroOrderDerivativeNodes, FirstOrderDerivativeNodes, ...
                SecondOrderDerivativeNodes);
        end
        
        %returns CoM position for a given time for pseudo static walk (when
        %CoM is always inside the support polygon); requires
        %.GenerateSplineTrajectory() to be called previously.
        function [r, dr, ddr] = PeudoStaticWalkCoM_All(obj, t)
            [r, dr, ddr] = obj.Trajectory.EvaluateAll(t);
            r(3) = obj.CoM_height;
            dr(3) = 0;
            ddr(3) = 0;
        end
        function r = PeudoStaticWalkCoM(obj, t)
            r = obj.Trajectory.EvaluateQ(t);
            r(3) = obj.CoM_height;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%% ZMP
        
        %Solves QP to generate CoM trajectory
        function [res_x, res_y, res_u] = SolveZMP_QP(obj, t, IC, Parameters)
            if nargin < 4
                Parameters = [];
            end
            if isfield(Parameters, 'TimeStep')
                TimeStep = Parameters.TimeStep;
            else
                TimeStep = 0.01;
            end
            if isfield(Parameters, 'Q_values')
                Q_values = Parameters.Q_values;
            else
                Q_values = 1;
            end
            if isfield(Parameters, 'R_values')
                R_values = Parameters.R_values;
            else
                R_values = 0.001;
            end
            if isfield(Parameters, 'NumberOfSteps')
                NumberOfSteps = Parameters.NumberOfSteps;
            else
                NumberOfSteps = 10;
            end
            if isfield(Parameters, 'verbose_value')
                verbose_value = Parameters.verbose_value;
            else
                verbose_value = 0;
            end
            
            Q = eye(2)*Q_values;
            R = eye(2)*R_values;
            
            A = [zeros(2), eye(2); zeros(2, 4)];
            B = [zeros(2); eye(2)];
            C = [eye(2), zeros(2)];
            D = eye(2)*(obj.CoM_height / 9.8);
            
            FinishTime = obj.Trajectory.NodeTimes(size(obj.Trajectory.NodeTimes, 1));
            
            x0 = [IC.x0; IC.y0; IC.dx0; IC.dy0];
            
            x = sdpvar(4, NumberOfSteps);
            y = sdpvar(2, NumberOfSteps);
            u = sdpvar(2, NumberOfSteps);
            
            Constraints = [x(:, 1) == x0 + TimeStep*(A*x0 + B*u(:, 1))];
            Objective = 0;
            
            for i = 1:(NumberOfSteps - 1)
                NewConstraint = [x(:, i+1) == x(:, i) + TimeStep*(A*x(:, i) + B*u(:, i))];
                Constraints = [Constraints, NewConstraint];
            end
            for i = 1:NumberOfSteps
                NewConstraint = [y(:, i) == C*x(:, i) + D*u(:, i)];
                Constraints = [Constraints, NewConstraint];
                
                t1 = t + (i - 1)*TimeStep;
                if t1 > FinishTime
                    t1 = FinishTime;
                end
                ZMP = obj.Trajectory.EvaluateQ(t1);
                e = y(:, i) - ZMP(1:2); 
                Objective = Objective + e'*Q*e + u(:, i)'*R*u(:, i);
            end
            
            if isempty(obj.Custom_YALMIP_opt)
                ops = sdpsettings('solver', 'quadprog', 'verbose', verbose_value);
            else
                ops = obj.Custom_YALMIP_opt;
            end
            
            % Solve the problem
            sol = optimize(Constraints, Objective, ops);
            
            % Analyze error flags
            if sol.problem == 0
                % Extract and display value
                res_x = value(x);
                res_y = value(y);
                res_u = value(u);
                
            else
                display('YALMIP failed to solve the problem');
                sol.info
                yalmiperror(sol.problem)
                
                res_x = []; res_y = []; res_u = [];
            end
        end
        
        %Iteratively solves QP to generate CoM trajectory
        function GenerateCoMTrajectoryArray(obj, IC, Parameters)
            
            if nargin < 3
                Parameters = [];
            end
            if ~isfield(Parameters, 'TimeStep')
                Parameters.TimeStep = 0.1;
            end
            if ~isfield(Parameters, 'NumberOfSteps')
                Parameters.NumberOfSteps = 30;
            end
            if isfield(Parameters, 'Visualize')
                Visualize = Parameters.Visualize;
            else
                Visualize = false;
            end
            
            N = size(obj.Trajectory.NodeTimes, 1);
            t0 = obj.Trajectory.NodeTimes(1);
            t1 = obj.Trajectory.NodeTimes(N);
            Count = floor((t1 - t0) / Parameters.TimeStep);    
            
            obj.CoM_trajectory_arrays.Position = zeros(Count, 3);
            obj.CoM_trajectory_arrays.Velocity = zeros(Count, 3);
            obj.CoM_trajectory_arrays.Acceleration = zeros(Count, 3);
            obj.CoM_trajectory_arrays.Time = zeros(Count, 1);
            
            for i = 1:Count
                t = t0 + i * Parameters.TimeStep;
                
                [x, y, u] = obj.SolveZMP_QP(t, IC, Parameters);
                
                r = x(1:2, 1);
                dr = x(3:4, 1);
                ddr = u(:, 1);
                
                if Visualize
                hold on;
                x = x'; y = y';
                plot3(x(:, 1), x(:, 2), zeros(size(x, 1), 1), 'Color', 'k');
                plot3(y(:, 1), y(:, 2), zeros(size(x, 1), 1), 'Color', 'g');
                drawnow;
                end
                
                
                obj.CoM_trajectory_arrays.Position(i, :) = [r; obj.CoM_height];
                obj.CoM_trajectory_arrays.Velocity(i, :) = [dr; 0];
                obj.CoM_trajectory_arrays.Acceleration(i, :) = [ddr; 0];
                obj.CoM_trajectory_arrays.Time(i) = t;
                
                IC.x0 = r(1); IC.y0 = r(2); IC.dx0 = dr(1); IC.dy0 = dr(2);
            end
            if Visualize
                plot3(obj.CoM_trajectory_arrays.Position(:, 1), obj.CoM_trajectory_arrays.Position(:, 2), ...
                    obj.CoM_trajectory_arrays.Position(:, 3), '--', 'LineWidth', 3, 'Color', 'y');
            end
        end
        
        %solves a single QP to generate the entire CoM trajectory
        function GenerateCoMTrajectoryArray_SingleQP(obj, IC, Parameters)
            if nargin < 3
                Parameters = [];
            end
            if ~isfield(Parameters, 'TimeStep')
                Parameters.TimeStep = 0.1;
            end
            if isfield(Parameters, 'Visualize')
                Visualize = Parameters.Visualize;
            else
                Visualize = false;
            end
            
            N = size(obj.Trajectory.NodeTimes, 1);
            t0 = obj.Trajectory.NodeTimes(1);
            t1 = obj.Trajectory.NodeTimes(N);
            Parameters.NumberOfSteps = floor((t1 - t0) / Parameters.TimeStep);   
            
            obj.CoM_trajectory_arrays.Position = zeros(Parameters.NumberOfSteps, 3);
            obj.CoM_trajectory_arrays.Velocity = zeros(Parameters.NumberOfSteps, 3);
            obj.CoM_trajectory_arrays.Acceleration = zeros(Parameters.NumberOfSteps, 3);
            obj.CoM_trajectory_arrays.Time = zeros(Parameters.NumberOfSteps, 1);            
            
            [x, y, u] = obj.SolveZMP_QP(t0, IC, Parameters);
            
            if ~isempty(x)
            x = x'; u = u'; y = y';
            obj.CoM_trajectory_arrays.Position(:, 1:2) = x(:, 1:2);
            obj.CoM_trajectory_arrays.Position(:, 3) = obj.CoM_height*ones(Parameters.NumberOfSteps, 1);
            obj.CoM_trajectory_arrays.Velocity(:, 1:2) = x(:, 3:4);
            obj.CoM_trajectory_arrays.Velocity(:, 3) = zeros(Parameters.NumberOfSteps, 1);
            obj.CoM_trajectory_arrays.Acceleration(:, 1:2) = u(:, 1:2);
            obj.CoM_trajectory_arrays.Acceleration(:, 3) = zeros(Parameters.NumberOfSteps, 1);
            
            for i = 1:Parameters.NumberOfSteps
                obj.CoM_trajectory_arrays.Time(i) = t0 + i * Parameters.TimeStep;
            end
            if Visualize
                hold on;
                plot3(obj.CoM_trajectory_arrays.Position(:, 1), obj.CoM_trajectory_arrays.Position(:, 2), ...
                    obj.CoM_trajectory_arrays.Position(:, 3), 'LineWidth', 3, 'Color', 'k');
                plot3(y(:, 1), y(:, 2), ...
                    zeros(size(y, 1), 1), '--', 'LineWidth', 3, 'Color', 'g');
            end
            else
                warning('no solution');
            end
        end
            
            
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%% Illustrations
        
        %Draws trajectory generated by .GenerateSplineTrajectory() method
        function DrawCoMTrajectory(obj, figure_handle, Parameters)
            
            if nargin < 3
                Parameters = [];
            end
            if nargin < 2
                figure_handle = [];
            end
            if isfield(Parameters, 'DrawTrajectoryOnly')
                DrawTrajectoryOnly = Parameters.DrawTrajectoryOnly;
            else
                DrawTrajectoryOnly = false;
            end
            
            N = size(obj.Trajectory.NodeTimes, 1);
            t0 = obj.Trajectory.NodeTimes(1);
            t1 = obj.Trajectory.NodeTimes(N);
            dt = 0.01;
            Count = floor((t1 - t0) / dt);
            
            Position = zeros(Count, 3);
            Time = zeros(Count, 1);
            
            for i = 1:Count
                t = t0 + i*dt;
                Position(i, :) = obj.PeudoStaticWalkCoM(t);
                Time(i) = t;
            end
            
            if isempty(figure_handle)
                fig = figure();
                fig.Color = 'w';
            else
                figure(figure_handle);
            end
            
            if DrawTrajectoryOnly
                plot3(Position(:, 1), Position(:, 2), Position(:, 3), 'LineWidth', 3); grid on;
            else
                subplot(2, 3, 1:3);
                plot3(Position(:, 1), Position(:, 2), Position(:, 3), 'LineWidth', 3); grid on;
                subplot(2, 3, 4);
                plot(Time, Position(:, 1), 'LineWidth', 3); grid on;
                subplot(2, 3, 5);
                plot(Time, Position(:, 2), 'LineWidth', 3); grid on;
                subplot(2, 3, 6);
                plot(Time, Position(:, 3), 'LineWidth', 3); grid on;
            end
        end
        
        
    end
end
   