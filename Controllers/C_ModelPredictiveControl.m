classdef C_ModelPredictiveControl < SRDControllerWrapper
    properties
        %%%%%%%%%%%%%%%%%%
        %%% functionality
        %
        %function handles
        generate_problem = [];
        
        solve_problem = [];
        
        %%%%%%%%%%%%%%%%%%
        %%% run time parameters
        
        %function handles
        ControlInput = [];
        
        %time step for the controller;
        TimeStep = [];
        
        %time step for the controller;
        TimeStepQP = [];
        
        %%%%%%%%%%%%%%%%%%
        %%% generation parameters
        
        ToOptimizeFunctions = true;
        
        %%%%%%%%%%%%%%%%%%
        %%% data
        
        %structures
        problem = [];
        solution = [];
        
        problem_sym = [];
        
        details = [];
        
        %%%%%%%%%%%%%%%%%%
        %%% settings
        
        Verbose = true;
        
        %%%%%%%%%%%%%%%%%%
        %%% class objects
        
        Math = [];
    end
    methods
        function obj = C_ModelPredictiveControl(SimulationEngine, varargin)
            obj = obj@SRDControllerWrapper();
            obj.SimulationEngine = SimulationEngine;
            
            Parser = inputParser;
            Parser.FunctionName = 'C_ModelPredictiveControl';
            Parser.addOptional('Type', 'Constrained');
            
            %optimization parameters
            Parser.addOptional('number_of_points', 5);
            Parser.addOptional('TimeStep', 0.1);
            Parser.addOptional('TimeStepQP', 0.3);
            
            Parser.addOptional('R', 0.01);
            Parser.addOptional('Q', 1);
            Parser.addOptional('Wl', 0.01);
            
            Parser.addOptional('u_dim', obj.SimulationEngine.Control_dof);
            Parser.addOptional('lambda_dim', obj.SimulationEngine.Constraint_dof);
            
            Parser.addOptional('options', optimoptions('quadprog', 'Display', 'none'));
            
            Parser.addOptional('Verbose', true);
            
            Parser.parse(varargin{:});
            
            %initilization
            obj.TimeStep = Parser.Results.TimeStep;
            obj.TimeStepQP = Parser.Results.TimeStepQP;
            obj.Verbose = Parser.Results.Verbose;
            
            obj.Math = MathClass;
            
            %generates quadprog problem
            function generate_problem_Constrained()
                %dimentions
                m = Parser.Results.number_of_points;
                n = obj.SimulationEngine.dof * 2;
                k = Parser.Results.u_dim;
                l = Parser.Results.lambda_dim;
                
                %dynamics and cost
                Q = OHfunction_GetCorrectWeights(Parser.Results.Q, n);
                R = OHfunction_GetCorrectWeights(Parser.Results.R, k);
                Wl = OHfunction_GetCorrectWeights(Parser.Results.Wl, l);
                
                Dynamics.H = sym('H', [n, n, m]);    assume(Dynamics.H, 'real');
                Dynamics.A = sym('A', [n, n, m]);    assume(Dynamics.A, 'real');
                Dynamics.B = sym('B', [n, k, m]);    assume(Dynamics.B, 'real');
                Dynamics.c = sym('c', [n, 1, m]);    assume(Dynamics.c, 'real');
                Dynamics.F = sym('F', [n, l, m]);    assume(Dynamics.F, 'real');%already transposed
                
                %decision vars
                Dynamics.x =            sym('x',          [n, m+1]); assume(Dynamics.x,         'real');
                Dynamics.x_desired =	sym('x_desired',  [n, m+1]); assume(Dynamics.x_desired, 'real');
                Dynamics.x0 =           sym('x0',         [n, 1]);   assume(Dynamics.x0,        'real');
                Dynamics.u =            sym('u',          [k, m]);   assume(Dynamics.u,         'real');
                Dynamics.lambda =       sym('lambda',     [l, m]);   assume(Dynamics.lambda,    'real');
                
                Dynamics.tau_desired =  sym('tau_desired',[n, m]);   assume(Dynamics.tau_desired, 'real');
                
                %time step
                dt = sym('dt'); assume(dt, 'real');
                Dynamics.dt = dt;
                
                
                %all variables
                var = [Dynamics.x(:); Dynamics.u(:); Dynamics.lambda(:)];
                
                %all coefficients
                Dynamics.all_coefficients = [Dynamics.H(:); 
                                             Dynamics.A(:); 
                                             Dynamics.B(:); 
                                             Dynamics.c(:); 
                                             Dynamics.F(:); 
                                             Dynamics.x_desired(:);
                                             Dynamics.x0(:);
                                             Dynamics.tau_desired(:);
                                             Dynamics.dt];
                if obj.Verbose
                    disp(['number of variables - ', num2str(length(var))]);
                    disp(['number of coefficients - ', num2str(length(Dynamics.all_coefficients))]);
                end
                
                %calculate dynamics constraints and quadratic cost
                Constraint = []; Cost = sym(0);
                for i = 1:m
                    if obj.Verbose; disp(['calculating constraints and cost for point #', num2str(i), ' out of ', num2str(m)]); end
                    
                    Hi = Dynamics.H(:, :, i);
                    Ai = Dynamics.A(:, :, i);
                    Bi = Dynamics.B(:, :, i);
                    ci = Dynamics.c(:, :, i);
                    Fi = Dynamics.F(:, :, i);
                    
                    xi =    Dynamics.x(:, i); 
                    xi1 =   Dynamics.x(:, i+1);
                    x_desiredi1 = Dynamics.x_desired(:, i+1);
                    ui =    Dynamics.u(:, i);
                    lambdai = Dynamics.lambda(:, i);
                    
                    tau_desiredi = Dynamics.tau_desired(:, i);
                    
                    %NewConstraint = (Hi - dt*Ai)*xi1 - Hi*xi - dt*Bi*ui - dt*Fi*lambdai - dt*ci;
                    NewConstraint = (1/dt)*Hi*(xi1 - xi) - (Ai*xi1 + Bi*ui + Fi*lambdai + ci);
                    Constraint = [Constraint; NewConstraint];
                    
                    %cost for control actons;
                    %notice the cost is calculated for the next step (i+1),
                    %cause it makes no sense to put cost on the initial
                    %position, it is fixed with an equality constraint
                    Cost_state = (x_desiredi1 - xi1)' * Q * (x_desiredi1 - xi1);
                    Cost_state = simplify(Cost_state);
                    
                    %cost for control actons
                    %tau_desired = Hi*(xi1 - xi) / dt - Ai*xi1 - ci;
                    %%delta_tau = tau_desiredi - Bi*ui - Fi*lambdai;
                    %%Cost_action = delta_tau' * R * delta_tau;
                    Cost_action = ui' * R * ui;
                    Cost_action = simplify(Cost_action);
                    
                    Cost_lambda = lambdai' * Wl * lambdai;
                    Cost_lambda = simplify(Cost_lambda);
                    
                    %total cost
                    Cost = Cost + Cost_state + Cost_action + Cost_lambda;
                end
                
                %add initial position constrain
                Constraint = [Constraint;
                              (Dynamics.x(:, 1) - Dynamics.x0)];
                
                          
                if obj.Verbose; disp('Generating linear constraints'); end
                %generate constrains
                [Aeq, beq] = OHfunction_generate_linear_constraints('LHS', Constraint, ...
                    'RHS', zeros(size(Constraint)), ...
                    'var', var, 'Numeric', false);
                
                if obj.Verbose; disp('Generating cost'); end
                %generate cost
                [cost_H, cost_f] = OHfunction_generate_quadratic_cost('Cost', Cost, ...
                    'var', var, 'Numeric', false);
                
                
                
                if obj.Verbose; disp('Generating g_control_MPC_H function'); end
                matlabFunction(cost_H, 'File', 'g_control_MPC_H', 'Vars', {Dynamics.all_coefficients}, ...
                    'Optimize', obj.ToOptimizeFunctions);
                
                if obj.Verbose; disp('Generating g_control_MPC_f function'); end
                matlabFunction(cost_f, 'File', 'g_control_MPC_f', 'Vars', {Dynamics.all_coefficients}, ...
                    'Optimize', obj.ToOptimizeFunctions);
                
                if obj.Verbose; disp('Generating g_control_MPC_Aeq function'); end
                matlabFunction(Aeq, 'File', 'g_control_MPC_Aeq', 'Vars', {Dynamics.all_coefficients}, ...
                    'Optimize', obj.ToOptimizeFunctions);
                
                if obj.Verbose; disp('Generating g_control_MPC_beq function'); end
                matlabFunction(beq, 'File', 'g_control_MPC_beq', 'Vars', {Dynamics.all_coefficients}, ...
                    'Optimize', obj.ToOptimizeFunctions);
                
                %write the problem
                obj.problem_sym.H = cost_H;
                obj.problem_sym.f = cost_f;
                obj.problem_sym.Aeq = Aeq;
                obj.problem_sym.beq = beq;
                
                obj.problem.solver = 'quadprog';
                obj.problem.options = Parser.Results.options;
                
                obj.details.TimeStep = Parser.Results.TimeStep;
                obj.details.m = m;
                obj.details.n = n;
                obj.details.k = k;
                obj.details.l = l;
                obj.details.Dynamics_sym = Dynamics;
                obj.details.var = var;
            end
            
            function Controller_Constrained(~, ~)
                
                SensorData = obj.SimulationEngine.SensorHandler.ReadCurrentData;
                
                %we find the time remained till the next LQR gains update
                remainder = 1;
                if ~isempty(obj.TimeStepQP)
                    remainder = mod(SensorData.t, obj.TimeStepQP);
                end
                
                %if the update is due, do it
                if isempty(obj.u) || (remainder == 0)
                    
                    m = obj.details.m;
                    n = obj.details.n;
                    k = obj.details.k;
                    l = obj.details.l;
                    
                    Dynamics.H = zeros(n, n, m);
                    Dynamics.A = zeros(n, n, m);
                    Dynamics.B = zeros(n, k, m);
                    Dynamics.c = zeros(n, 1, m);
                    Dynamics.F = zeros(n, l, m); %already transposed
                
                    obj.solution.Time = zeros(1, m+1);
                    Dynamics.x_desired = zeros(n, m+1);
                    
                    Dynamics.x0 = SensorData.x;
                    u_zero = zeros(obj.details.k, 1);
                    
                    %desired trajectory; generated beforehand because below
                    %we use finite differences forward and implicit Euler
                    %scheme, both require state one step ahead
                    for i = 1:obj.details.m+1
                        t = SensorData.t + (i - 1) * obj.TimeStep;
                        
                        [desired_q, desired_v, ~] = obj.ControlInput(t);
                        
                        obj.solution.Time(1, i) = t;
                        Dynamics.x_desired(:, i) = [desired_q; desired_v];
                    end
                    
                    for i = 1:obj.details.m
                        
                        x_desired_i = Dynamics.x_desired(:, i);
                        x_desired_i1 = Dynamics.x_desired(:, i+1);
                        
                        %linearize the model around the current point
                        LinearModel = obj.SimulationEngine.GetLinearization(x_desired_i, u_zero);
                        
                        tau_desired_i = LinearModel.H * (x_desired_i1 - x_desired_i) / obj.TimeStep - ...
                            LinearModel.A * x_desired_i1;
                        
                        Dynamics.x_desired(:, i) = x_desired_i;
                        
                        Dynamics.H(:, :, i) = LinearModel.H;
                        Dynamics.A(:, :, i) = LinearModel.A;
                        Dynamics.B(:, :, i) = LinearModel.B;
                        Dynamics.c(:, 1, i) = LinearModel.c;
                        
                        Dynamics.F(:, :, i) = [g_dynamics_LagrangeMultiplier_ConstraintJacobian(desired_q)'; 
                                               zeros(obj.SimulationEngine.dof, l)];
                        
                        Dynamics.tau_desired(:, i) = tau_desired_i;
                    end
                    
                    Dynamics.all_coefficients = [Dynamics.H(:);
                                                 Dynamics.A(:);
                                                 Dynamics.B(:);
                                                 Dynamics.c(:);
                                                 Dynamics.F(:);
                                                 Dynamics.x_desired(:);
                                                 Dynamics.u_desired(:);
                                                 Dynamics.lambda_desired(:);
                                                 Dynamics.x0(:);
                                                 obj.TimeStep];
                    
                    obj.problem.H = g_control_MPC_H(Dynamics.all_coefficients);
                    obj.problem.f = g_control_MPC_f(Dynamics.all_coefficients);
                    obj.problem.Aeq = g_control_MPC_Aeq(Dynamics.all_coefficients);
                    obj.problem.beq = g_control_MPC_beq(Dynamics.all_coefficients);
                    
                    
                    [var, fval, exitflag] = quadprog(obj.problem);
                    obj.solution.var = var;
                    obj.solution.fval = fval;
                    obj.solution.exitflag = exitflag;
                    
                    index1 = n*(m+1);
                    index2 = index1 + k*m;
                    index3 = index2 + l*m;
                    
                    X = obj.solution.var(1:index1);
                    U = obj.solution.var((index1+1):index2);
                    L = obj.solution.var((index2+1):index3);
                    
                    obj.solution.X = reshape(X, [n, m+1]);
                    obj.solution.U = reshape(U, [k, m]);
                    obj.solution.L = reshape(L, [l, m]);
                    obj.u = obj.solution.U(:, 1);
                    
                    obj.details.Dynamics_num = Dynamics;
                else
                    if ~isempty(obj.solution)
                        Index = obj.Math.FindPlaceInArray(obj.solution.Time(1:(obj.details.m-1)), ...
                            SensorData.t, 'Closest smaller');
                        obj.u = obj.solution.U(:, Index);
                    end
                end
            end
            
            switch Parser.Results.Type
                case {'Constrained'}
                    obj.Controller          = @Controller_Constrained;
                    obj.generate_problem    = @generate_problem_Constrained;
                    obj.solve_problem       = [];
                otherwise
                    error('Invalid controller type');
            end
            
        end
    end
end