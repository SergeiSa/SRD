classdef C_TrajectoryPlanner < SRDControllerWrapper
    properties
        %functions
        generate_problem = [];
        solve_problem = [];
        aproximate = [];
        evaluate = [];
        InverseDynamics = [];
        InverseKinematics = [];
        
        %data
        details = [];
        problem = [];
        solution = [];
        
        %Trajectory end time
        TimeEnd;
        
        %classes
        Approximator;
    end
    methods
        
        function obj = C_TrajectoryPlanner(SimulationEngine, varargin)
            obj = obj@SRDControllerWrapper();
            obj.SimulationEngine = SimulationEngine;
            
            x0 = [obj.SimulationEngine.IC.q; obj.SimulationEngine.IC.v];
            
            Parser = inputParser;
            Parser.FunctionName = 'GetTrajectoryPlanner';
            Parser.addOptional('Type', 'Boundary');
            
            %optimization parameters
            Parser.addOptional('number_of_points', 10);
            Parser.addOptional('TimeStep', 0.1);
            Parser.addOptional('LinearModel', ...
                obj.SimulationEngine.GetLinearization(x0, zeros(obj.SimulationEngine.Control_dof, 1)));
            Parser.addOptional('R', eye(obj.SimulationEngine.Control_dof));
            Parser.addOptional('Q', eye(obj.SimulationEngine.dof * 2));
            Parser.addOptional('options', optimoptions('quadprog', 'Display', 'iter'));
            Parser.addOptional('UseImplicitEuler', true);
            
            %minimizing movements between neighbouring points
            Parser.addOptional('neighbour_Q', zeros(obj.SimulationEngine.dof * 2));
            
            %boundary
            Parser.addOptional('u_dim', obj.SimulationEngine.Control_dof);
            Parser.addOptional('S_start', eye(obj.SimulationEngine.dof * 2));   %end position is constrained as
            Parser.addOptional('b_start', x0);                              %S_start * x = b_start;
            Parser.addOptional('S_goal',  eye(obj.SimulationEngine.dof * 2));   %end position is constrained as
            Parser.addOptional('b_goal',  x0);                              %S_goal * x = b_goalx
            
            Parser.parse(varargin{:});
            
            %generates quadprog problem
            function generate_problem_Boundary()
                %dimentions
                m = Parser.Results.number_of_points;
                n = obj.SimulationEngine.dof * 2;
                k = Parser.Results.u_dim;
                
                %time step
                dt = Parser.Results.TimeStep;
                
                %dynamics and cost
                I = eye(n);
                R = Parser.Results.R;
                Q = Parser.Results.Q;
                neighbour_Q = Parser.Results.neighbour_Q;
                Model_A = Parser.Results.LinearModel.A;
                Model_B = Parser.Results.LinearModel.B;
                Model_c = Parser.Results.LinearModel.c;
                
                %decision vars
                x = sym('x', [n, m]); assume(x, 'real');
                u = sym('u', [k, m]); assume(u, 'real');
                
                %calculate dynamics constraints and quadratic cost
                Constraint = []; Cost = sym(0);
                for i = 1:(m-1)
                    if Parser.Results.UseImplicitEuler
                        NewConstraint = [(I - dt*Model_A)*x(:, i+1) - x(:, i) - (dt*Model_B)*u(:, i) - (dt*Model_c)];
                    else
                        NewConstraint = [x(:, i+1) - (I + dt*Model_A)*x(:, i) - (dt*Model_B)*u(:, i) - (dt*Model_c)];
                    end
                    Constraint = [Constraint; NewConstraint];
                    
                    %cost for defferences between neighbour points
                    delta = x(:, i+1) - x(:, i);
                    Cost_neighbour = delta' * neighbour_Q * delta;
                    
                    %cost for control actons
                    Cost_action = u(:, i)' * R * u(:, i);
                    
                    %total cost
                    Cost = Cost + Cost_action + Cost_neighbour;
                end
                
                %add initial position constrain
                Constraint = [Constraint;
                              Parser.Results.S_start * (x(:, 1) - Parser.Results.b_start)];
                
                %add goal position cost
                error_goal = Parser.Results.S_goal * (x(:, m) - Parser.Results.b_goal);
                if size(Q, 2) ~= size(error_goal, 1)
                    Q = Parser.Results.S_goal * Q * Parser.Results.S_goal';
                end
                Cost = Cost + error_goal' * Q * error_goal;
                
                %generate constrains
                var = [x(:); u(:)];
                [Aeq, beq] = OHfunction_generate_linear_constraints('LHS', Constraint, ...
                    'RHS', zeros(size(Constraint)), ...
                    'var', var);
                
                %generate cost
                [cost_H, cost_f] = OHfunction_generate_quadratic_cost('Cost', Cost, 'var', var);
                
                %write the problem
                obj.problem.H = cost_H;
                obj.problem.f = cost_f;
                obj.problem.Aeq = Aeq;
                obj.problem.beq = beq;
                obj.problem.solver = 'quadprog';
                obj.problem.options = Parser.Results.options;
                
                obj.details.LinearModel = Parser.Results.LinearModel;
                obj.details.TimeStep = Parser.Results.TimeStep;
                obj.details.m = m;
                obj.details.n = n;
                obj.details.k = k;
                obj.details.Q = Q;
            end
            
            
            function solve_problem_Boundary()
                
                [var, fval, exitflag] = quadprog(obj.problem);
                obj.solution.var = var;
                obj.solution.fval = fval;
                obj.solution.exitflag = exitflag;
                
                %dimentions
                m = obj.details.m;
                n = obj.details.n;
                k = obj.details.k;
                
                index = n * m; 
                q_index = 1:obj.SimulationEngine.dof; 
                v_index = (obj.SimulationEngine.dof+1):(2*obj.SimulationEngine.dof);
                a_index = (obj.SimulationEngine.dof+1):(2*obj.SimulationEngine.dof);
                
                obj.solution.x = reshape(var(1:index), n, m);
                obj.solution.u = reshape(var((index+1):length(var)), k, m);
                obj.solution.t = Parser.Results.TimeStep * (0:1:(m-1));
                
                obj.solution.q = obj.solution.x(q_index, :);
                obj.solution.v = obj.solution.x(v_index, :);
                
                obj.solution.a = zeros(size(obj.solution.q));
                for i = 1:size(obj.solution.x, 2)
                    dx = obj.details.LinearModel.A * obj.solution.x(:, i) + ...
                         obj.details.LinearModel.B * obj.solution.u(:, i) + ...
                         obj.details.LinearModel.c;
                    obj.solution.a(:, i) = dx(a_index);
                end
                
                obj.TimeEnd = obj.solution.t(end);
            end
            
            function aproximate_Default(NumberOfSegments, PolynomialDegree)
                Time = obj.solution.t';
                
                obj.Approximator.Position = TPPolynomialDataApproximation;
                obj.Approximator.Position.Approximator(Time, obj.solution.q', NumberOfSegments, PolynomialDegree);
                
                obj.Approximator.Velocity = TPPolynomialDataApproximation;
                obj.Approximator.Velocity.Approximator(Time, obj.solution.v', NumberOfSegments, PolynomialDegree);
                
                obj.Approximator.Acceleration = TPPolynomialDataApproximation;
                obj.Approximator.Acceleration.Approximator(Time, obj.solution.a', NumberOfSegments, PolynomialDegree);
                
                obj.Approximator.ControlAction = TPPolynomialDataApproximation;
                obj.Approximator.ControlAction.Approximator(Time, obj.solution.u', NumberOfSegments, PolynomialDegree);
            end
            
            function [q, v, a, u] = evaluate_Default(SensorData)
                q = obj.Approximator.Position.Evaluate(SensorData.t);
                v = obj.Approximator.Velocity.Evaluate(SensorData.t);
                a = obj.Approximator.Acceleration.Evaluate(SensorData.t);
                u = obj.Approximator.ControlAction.Evaluate(SensorData.t);
            end
            
            function Output = InverseDynamics_Default(SensorData)
               Output.u0 = obj.Approximator.ControlAction.Evaluate(SensorData.t);
            end
            
            function [q, v, a] = InverseKinematics_Default(t)
                q = obj.Approximator.Position.Evaluate(t);
                v = obj.Approximator.Velocity.Evaluate(t);
                a = obj.Approximator.Acceleration.Evaluate(t);
            end
            
            switch Parser.Results.Type
                case {'Boundary'}
                    obj.Controller          = @PDcontroller;
                    obj.generate_problem    = @generate_problem_Boundary;
                    obj.solve_problem       = @solve_problem_Boundary;
                    obj.aproximate          = @aproximate_Default;
                    obj.evaluate            = @evaluate_Default;
                    obj.InverseDynamics     = @InverseDynamics_Default;
                    obj.InverseKinematics   = @InverseKinematics_Default;
                otherwise
                    error('Invalid controller type');
            end
            
        end
    end
end