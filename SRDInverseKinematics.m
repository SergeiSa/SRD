%This class provides functions for inverce kinematics
%last update 6.10.16
classdef SRDInverseKinematics < handle
    properties
        
        %%%%%%%%%%%%%%%%%%%%%%%%
        %%% Symbolics and setup
        %%%%%%%%%%%%%%%%%%%%%%%%
        
        JointSpace_dof;
        % the dimentions of the joint space (number of generalized
        % coordinates of the robot)
        
        SolveForDerivatives = true;
        %If true, the IK solution for v = dq/dt and a = dv/dt will also be
        %provided
        
        ToSimplify = true;
        %If true, the class will attempt to simplify the symbolic
        %expressions where possible
        
        UseParallelizedSimplification = false;
        %If true, instead of normal MATLAB Simplify function a parallelized
        %one will be used
        
        ToOptimizeFunctions = true;
        %If false, when generating functions the automatic optimization
        %will be turned off. Can save time for big functions.
        
        %%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%%%%%%%%%%%%%%%%%
        %%% Task settings
        %%%%%%%%%%%%%%%%%%%%%%%%
        
        TaskSpace_dof;
        % the dimentions of the task space (number of components of the
        % task)
        
        DesiredTask;
        % function handle for a function that provides a desired value of
        % the task. It should have the following structure:
        % [r; dr; ddr] = DesiredTask(t)
        % where r is the task, dr is its derivative, ddr is its second
        % derivative
        
        DesiredTask_FirstDerivative = [];
        DesiredTask_SecondDerivative = [];
        % function handles for functions that provide desired values of
        % the task's first and second derivatives. If you provide both
        % the class will use them instead of DesiredTask when it only needs
        % the derivatives, potentially saving computations. Especially
        % important if the IK is solved only once and then saved; then
        % evaluation of the DesiredTask itself is never needed, hence
        % providing these two functions reduces the computations by about
        % 30%.
        
        %%%%%%%%%%%%%%%%%%%%%%%%
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%
        %%% Full IK solution settings
        %%%%%%%%%%%%%%%%%%%%%%%%
        
        TimeStart = 0;
        %time where IK trajectory starts
        
        TimeEnd = 10;
        %time where IK trajectory ends
        
        TotalTime = 10;
        %time length of the IK trajectory
        
        dt = 0.01;
        % Time step that will be used when solving the inverse kinematics.
        
        %%%%%%%%%%%%%%%%%%%%%%%%
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%
        %%% Optimization-based inverse kinematics (one iteration IK
        %%% settings)
        %%%%%%%%%%%%%%%%%%%%%%%%
        
        InitialGuess;
        % If non-empty initial guess is provided the optimization inverse
        % kinematics (OIK) will use it as a starting value
        % It needs to have n elements, where n is the dof of the robot.
        
        SolverType = 'lsqnonlin';
        %determines what optimization solver will be used to solve IK
        
        ValidSolverTypes = {'lsqnonlin', 'fmincon', 'ProjectionNonlinSolver'}
        %List of the valid values for property .SolverType
        
        RandomizerMagnitudeForOIK = 0.1;
        % determines the magnitude for randomizer used in
        % optimization-based IK solution. It means that if a solution is no
        % acceptable (the solver produced a bad local minimum) its result
        % will be pererbed by a random number added to it, and the problem
        % will be solved again. RandomizerMagnitudeForOIK defines
        % element-wise range of the perturbation
        
        MaxRandomizerAttempts = 7;
        % determines how many attampts to get out of the local minima will
        % be made
        
        LocalMinimumCriteria = 0.001;
        % the threshold for the error in IK solution, anything higher than
        % that will be considered a local minimum
        
        %%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%%%%%%%%%%%%%%%%%
        %%% Approximation
        %%%%%%%%%%%%%%%%%%%%%%%%
        PolynomialApproximation;
        %an object of PolynomialDataApproximationClass class, contains the
        %polynomial approximation for IK solution
        
        LookupTable;
        LookupTable_dt = 0.001;
        %.LookupTable is a structure that stores tables with IK solution,
        %with .LookupTable_dt time step. It can be used to avoid
        %polynomial evaluations and other calculations.
        %LookupTable.Time - contains time
        %LookupTable.Position - contains gen. coordinates
        %LookupTable.Velocity - contains gen. velocities
        %LookupTable.Acceleration - contains gen. coordinates
        
        use_pinv = false;
        %if true, the algorithms will use pinv instead of \ for solving
        %linear eq.
        
        %%%%%%%%%%%%%%%%%%%%%%%%
    end
    methods
        %Class constructor
        function obj = SRDInverseKinematics()
            
        end
        
        % This function sets up the inverse kinematics problem.
        % q - symbolic vector of gen. coordinates,
        % Task - symbolic expression of the task.
        %
        % the inverse kinematics can be given as the following problem:
        % find q, such that
        % R(q) = Rdesired, where
        % R(q) is the Task and Rdesired is a time function of the same
        % dimentions.
        % In practice a similar problem is solved instead:
        % q = argmin((R(q) - Rdesired)'*(R(q) - Rdesired))
        %
        % For the derivatives of q the following methods are used:
        % v = inv(J1)*dR/dt, where
        % v = dq/dt, J1 = dR/dq;
        % and
        % a = inv(J1)*(d^2R/dt^2 - J2*v), where
        % a = d^2q/dt^2, J2 = d(dR/dt)/dq;
        function IKsetup(obj, q, Task)
            tic;
            
            obj.JointSpace_dof = size(q, 1);
            obj.TaskSpace_dof = size(Task, 1);
            
            v = sym('v', [obj.JointSpace_dof, 1]); assume(v, 'real');
            a = sym('a', [obj.JointSpace_dof, 1]); assume(a, 'real');
            
            if obj.ToSimplify
                Task = simplify(Task);
            end
            matlabFunction(Task, 'File', 'g_InverseKinematics_Task', 'Vars', {q});
            
            %Here we generate jacobians for the IK problem
            if obj.SolveForDerivatives
                J1 = jacobian(Task, q);
                if obj.ToSimplify
                    disp('Started simplifying first inverse kinematics jacobian');
                    if obj.UseParallelizedSimplification
                        Math = MathClass;
                        J1 = Math.ParallelizedSimplification(J1, 'J1');
                    else
                        J1 = simplify(J1);
                    end
                    disp('* Finished simplifying first inverse kinematics jacobian');
                end
                dTask = J1*v;
                J2 = jacobian(dTask, q);
                if obj.ToSimplify
                    disp('Started simplifying second inverse kinematics jacobian');
                    if obj.UseParallelizedSimplification
                        Math = MathClass;
                        J2 = Math.ParallelizedSimplification(J2, 'J2');
                    else
                        J2 = simplify(J2);
                    end
                    disp('* Finished simplifying second inverse kinematics jacobian');
                end
                
                disp('Starting writing function for J1');
                matlabFunction(J1, 'File', 'g_InverseKinematics_J1', 'Vars', {q}, 'Optimize', obj.ToOptimizeFunctions);
                disp('Starting writing function for J2');
                matlabFunction(J2, 'File', 'g_InverseKinematics_J2', 'Vars', {q, v}, 'Optimize', obj.ToOptimizeFunctions);
                disp('* Finished generating inverse kinematics functions'); disp(' ');
            end
            toc
        end
        
        %This function solves inverse kinematics using fmincon
        %optimization. It only solves it for one point of time t. You need
        %to provide a good guess for the solution previous_q.
        %DesiredTask is function handle for the function that gives you
        %desired value of the task, taking time as input.
        %problem - see the description of the chosen solver (field
        %.SolverType of the class)
        function found_q = IKviaOptimizationOneIteration(obj, t, previous_q, DesiredTask, problem)
            
            if nargin < 5
                problem = [];
            end
            if isempty(problem)
                define_problem = true;
            else
                define_problem = false;
            end
            
            DesiredTaskValue = DesiredTask(t);
            
            function Error = ObjectiveFunction_lsqnonlin(q)
                Error = DesiredTaskValue - g_InverseKinematics_Task(q);
            end
            
            function Value = ObjectiveFunction_fmincon(q)
                Error = DesiredTaskValue - g_InverseKinematics_Task(q);
                Value = dot(Error, Error);
            end
            
            function Value = ObjectiveFunction_ProjectionNonlinSolver(q)
                Value = g_InverseKinematics_Task(q);
            end
            
            function Value = ObjectiveFunction_quadprog(q)
                Value = 0.5*q'*problem.H*q + problem.f*q;
            end
            
            %Set up options that only need to be set once.
            switch obj.SolverType
                case 'lsqnonlin'
                    ObjectiveFunction = @ObjectiveFunction_lsqnonlin;
                    if define_problem
                        problem.options = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt', ...
                            'Display', 'none', 'Jacobian', 'off');
                    end
                    problem.solver = 'lsqnonlin';
                case 'fmincon'
                    ObjectiveFunction = @ObjectiveFunction_fmincon;
                    if define_problem
                        problem.options = optimoptions('fmincon', 'Display', 'none', 'Algorithm', 'interior-point');
                    end
                    problem.solver = 'fmincon';
                case 'quadprog'
                    if define_problem
                        problem.options = optimoptions('quadprog', 'Display', 'none');
                    end
                    problem.solver = 'quadprog';
                    
                    J1 = g_InverseKinematics_J1(previous_q);
                    if isfield(problem, 'WeightMatrix')
                        W = problem.WeightMatrix;
                    else
                        W = eye(size(J1, 1));
                    end
                    task_evaluated = g_InverseKinematics_Task(previous_q);
                    c = task_evaluated - J1*previous_q;
                    
                    %coefficient determinimg how much the new solution
                    %should gravitate to the old one
                    w1 = 0.01;
                    
                    %generate the quadratic cost matrix
                    H = J1' * W * J1; %the cost
                    H = (H + H')/2;   %making it symmetric
                    H = H + w1*eye(size(H, 1)); %adding the terms that pull it to the previous solution
                    
                    %generate linear cost vector
                    f = (c - DesiredTaskValue)' * W * J1; %the cost
                    f = f - w1*previous_q'; %adding the terms that pull it to the previous solution
                    
                    problem.H = H;
                    problem.f = f;
                    ObjectiveFunction = @ObjectiveFunction_quadprog;
                    
                case 'ProjectionNonlinSolver'
                    ObjectiveFunction = @ObjectiveFunction_ProjectionNonlinSolver;
                    problem.t = t;
                    problem.FirstJacobian = @g_InverseKinematics_J1;
                    problem.SecondJacobian = @g_InverseKinematics_J2;
                    problem.GetTask = DesiredTask;
                    problem.TimeStep = obj.dt;
                    
                    PNS = TPNonlinEqSolver_Projection();
                otherwise
                    error('Invalid solver type');
            end
            
            problem.objective = ObjectiveFunction;
            
            %This function defines stopping criteria for iterative IK
            %solving algorithm. The algorithms attempts to climb out of a
            %local minimum using no more than .MaxRandomizerAttempts
            %attempts. The criteria for being in a local minimum is
            %objective function havin a norm bigger than
            %.LocalMinimumCriteria value
            function ToContinue = LoopCriteria(found_q, IterationNumber)
                if isempty(found_q)
                    ToContinue = true;
                else
                    if IterationNumber == 1
                        ToContinue = true;
                    else
                        if IterationNumber <= obj.MaxRandomizerAttempts
                            if norm(ObjectiveFunction(found_q)) > obj.LocalMinimumCriteria
                                ToContinue = true;
                            else
                                ToContinue = false;
                            end
                        else
                            ToContinue = false;
                        end
                    end
                end
            end
            
            found_q = previous_q;
            IterationNumber = 1;
            %Start solving the IK
            while LoopCriteria(found_q, IterationNumber)
                
                problem.x0 = found_q;
                switch obj.SolverType
                    case 'lsqnonlin'
                        found_q = lsqnonlin(problem);
                    case 'fmincon'
                        found_q = fmincon(problem);
                    case 'quadprog'
                         [found_q, ~, ~] = quadprog(problem);
                         %disp(num2str(exitflag));
                    case 'ProjectionNonlinSolver'
                        found_q = PNS.Solve(problem);
                end
                problem.x0 = found_q + obj.RandomizerMagnitudeForOIK*( rand(size(found_q)) - 0.5*ones(size(found_q)) );
                IterationNumber = IterationNumber + 1;
            end
        end
        
        
        
        
        %This function solves inverse kinematics using optimization
        %DesiredTask is function handle for the function that gives you
        %desired value of the task, taking time as input.
        %problem - see the description of the chosen solver (field
        %.SolverType of the class)
        %
        %The output of the method is a structure of arrays:
        %- Output.Time - is a vector of time
        %- Output.IK - is an array with rows corresponding to the IK
        %solutions for the time moments in the corresponding rows of .Time
        %- Output.TaskTrajectories - is an array with rows corresponding to
        %the desired task trajectories for the time moments in the
        %corresponding rows of .Time
        %- Output.ObtainedTaskTrajectories - is an array with rows
        %corresponding to the obtained task trajectories for the time
        %moments in the corresponding rows of .Time
        %- Output.SaveFirstJacobianConditionNumber - is a vector of First
        %Jacobian condition number values for corresponding rows of .Time
        function Output = IKviaOptimization(obj, DesiredTask, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDInverseKinematics.IKviaOptimization';
            Parser.addOptional('problem', []);
            Parser.addOptional('OutputDesiredTaskTrajectories', false);
            Parser.addOptional('OutputFirstJacobianConditionNumber', false);
            Parser.parse(varargin{:});
            
            %Check if the optimization was provided with a starting point
            if isempty(obj.InitialGuess)
                error('No initial guess was provided for the inverse kinematics');
            else
                q = obj.InitialGuess;
            end
            
            Count = floor(obj.TotalTime/obj.dt) + 1;
            
            %Preallocation
            Output.Time = zeros(Count, 1);
            Output.IK = zeros(Count, obj.JointSpace_dof);
            if Parser.Results.OutputDesiredTaskTrajectories
                Output.TaskTrajectories = zeros(Count, obj.TaskSpace_dof);
            end
            if Parser.Results.OutputFirstJacobianConditionNumber
                Output.FirstJacobianConditionNumber = zeros(Count, 1);
            end
            
            for i = 1:Count
                disp(['calculating ', num2str(i), ' out of ', num2str(Count)]);
                t = (i - 1)*obj.dt + obj.TimeStart;

                q = obj.IKviaOptimizationOneIteration(t, q, DesiredTask, Parser.Results.problem);
                
                Output.Time(i, 1) = t;
                Output.IK(i, :) = q';
                
                if Parser.Results.OutputDesiredTaskTrajectories
                    Output.TaskTrajectories(i, :) = DesiredTask(t)';
                    Output.ObtainedTaskTrajectories(i, :) = g_InverseKinematics_Task(q)';
                end
                if Parser.Results.OutputFirstJacobianConditionNumber
                    Output.FirstJacobianConditionNumber(i) = cond(g_InverseKinematics_J1(q));
                end
            end
        end
        
        % This function approximates the IK array with polynomials using
        % PolynomialDataApproximationClass functionality
        % -Input is the output of the function IKviaOptimization
        % -PolynomialDegree, determines the degree of polynomials that will
        % be used for the approximation.
        %
        % The function generates .PolynomialApproximation (object of PolynomialDataApproximationClass)
        % that contains the approximation and is used in
        % .EvaluatePolynomialApproximation() method
        function IK_PolynomialApproximation(obj, Input, PolynomialDegree, NumberOfSegments)
            obj.PolynomialApproximation = TPPolynomialDataApproximation;
            obj.PolynomialApproximation.Approximator(Input.Time, Input.IK, NumberOfSegments, PolynomialDegree);
        end
        
        % This function updates the field .ApproximationPolynomials using
        % IK_SinglePolynomialApproximation method. It calls
        % IKviaOptimization to get the IK solution that is
        % approximated by IK_SinglePolynomialApproximation
        function SolveAndApproximate(obj, DesiredTask, PolynomialDegree, NumberOfSegments, problem)
            if nargin < 5
                problem = [];
            end
            obj.DesiredTask = DesiredTask;
            obj.TotalTime = obj.TimeEnd - obj.TimeStart;
            
            IKOutput = obj.IKviaOptimization(DesiredTask, 'problem', problem);
            obj.IK_PolynomialApproximation(IKOutput, PolynomialDegree, NumberOfSegments);
        end
        
        %This function provides IK solution and the desired gen. velocities
        %and accelerations. Method SetupIKWithSinglePolynomialApproximation
        %needs to be called before using this function
        function q = EvaluatePolynomialApproximationQ(obj, t)
            q = obj.PolynomialApproximation.Evaluate(t);
        end
        
        %This function provides IK solution and the desired gen. velocities
        %and accelerations. Method SetupIKWithSinglePolynomialApproximation
        %needs to be called before using this function
        function [q, v, a] = EvaluatePolynomialApproximation(obj, t)
            q = obj.PolynomialApproximation.Evaluate(t);
            
            if isempty(obj.DesiredTask_FirstDerivative) || isempty(obj.DesiredTask_SecondDerivative)
                [~, dr, ddr] = obj.DesiredTask(t);
                %If we don't have both .DesiredTask_FirstDerivative and
                %.DesiredTask_SecondDerivative functions, we just use
                %.DesiredTask to get the derivatives.
            else
                dr = obj.DesiredTask_FirstDerivative(t);
                ddr = obj.DesiredTask_SecondDerivative(t);
                %if we do have them, then here we get the first and
                %second derivative
            end
            
            %IK jacobians
            J1 = g_InverseKinematics_J1(q);
            
            if obj.use_pinv
                v = pinv(J1) * dr;
            else
                v = J1 \ dr;
            end
            J2 = g_InverseKinematics_J2(q, v);
            
            %Solving for velocities and accelerations
            b = [dr; ddr];
            A = [J1, zeros(size(J1));
                J2, J1];
            
            if obj.use_pinv
                x = pinv(A) * b;
            else
                x = A \ b;
            end
            
            v = x(1:obj.JointSpace_dof);
            a = x((obj.JointSpace_dof+1):obj.JointSpace_dof*2);
        end
        
        %This function plots the graphs for desired gen. coordinates, gen.
        %velocities and gen. accelerations obtained with
        %EvaluatePolynomialApproximation method for the time interval
        %specified in the field .Time
        function PlotGraphsFromEvaluatePolynomialApproximation(obj)
            Count = floor(obj.TotalTime/obj.dt) + 1;
            
            Output.Time = zeros(Count, 1);
            Output.Position = zeros(Count, obj.JointSpace_dof);
            Output.Velocity = zeros(Count, obj.JointSpace_dof);
            Output.Acceleration = zeros(Count, obj.JointSpace_dof);
            
            for i = 1:Count
                t = (i - 1)*obj.dt + obj.TimeStart;
                [q, v, a] = obj.EvaluatePolynomialApproximation(t);
                Output.Time(i) = t;
                Output.Position(i, :) = q';
                Output.Velocity(i, :) = v';
                Output.Acceleration(i, :) = a';
            end
            
            figure('Color', 'w', 'Name', 'Inverse kinematics solution');
            subplot(2, 2, 1:2);
            SRDgraphic_PlotPositions(Output, 'NewFigure', false); 
            title('Inverse kinematics - position');
            subplot(2, 2, 3);
            SRDgraphic_PlotVelocities(Output, 'NewFigure', false); 
            title('Inverse kinematics - velocity');
            subplot(2, 2, 4);
            SRDgraphic_PlotAccelerations(Output, 'NewFigure', false); 
            title('Inverse kinematics - acceleration');
        end
        
        %this function generates a lookup table for LookupTableSolution
        function GenerateLookupTable(obj)
            Count = floor((obj.TimeEnd - obj.TimeStart) / obj.LookupTable_dt);
            
            obj.LookupTable.Time = zeros(Count, 1);
            obj.LookupTable.Position = zeros(Count, obj.JointSpace_dof);
            obj.LookupTable.Velocity = zeros(Count, obj.JointSpace_dof);
            obj.LookupTable.Acceleration = zeros(Count, obj.JointSpace_dof);
            
            for i = 1:Count
                
                t = obj.TimeStart + i*obj.LookupTable_dt;
                [q, v, a] = EvaluatePolynomialApproximation(obj, t);
                
                obj.LookupTable.Time(i) = t;
                obj.LookupTable.Position(i, :) = q';
                obj.LookupTable.Velocity(i, :) = v';
                obj.LookupTable.Acceleration(i, :) = a';
            end
        end
        
        %this function provides IK solution by using pre-generated lookup
        %table, useful if you need to speed-up the computations
        function [q, v, a] = LookupTableSolution(obj, t)
            index = floor((t - obj.TimeStart) / obj.LookupTable_dt);
            
            q = obj.LookupTable.Position(index, :)';
            v = obj.LookupTable.Velocity(index, :)';
            a = obj.LookupTable.Acceleration(index, :)';
        end
        
    end
end